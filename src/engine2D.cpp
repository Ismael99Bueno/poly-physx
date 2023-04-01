#include "engine2D.hpp"
#include "ode2D.hpp"
#include "perf.hpp"
#include <random>

namespace ppx
{
    engine2D::engine2D(const rk::butcher_tableau &table,
                       const std::size_t allocations) : m_collider(engine_key(), &m_entities, 2 * allocations),
                                                        m_compeller(engine_key(), &m_entities, allocations, &m_callbacks),
                                                        m_integ(table), m_callbacks(engine_key())
    {
        m_entities.reserve(allocations);
        m_integ.state().reserve(6 * allocations);
    }

    void engine2D::retrieve(const std::vector<float> &vars_buffer)
    {
        PERF_FUNCTION()
        for (std::size_t i = 0; i < m_entities.size(); i++)
            m_entities[i].retrieve(vars_buffer);
    }

    void engine2D::retrieve() { retrieve(m_integ.state().vars()); }

    bool engine2D::raw_forward(float &timestep)
    {
        const bool valid = m_integ.raw_forward(m_elapsed, timestep, *this, ode);
        register_forces_into_entities();
        reset_forces();
        retrieve();
        return valid;
    }
    bool engine2D::reiterative_forward(float &timestep, const std::size_t reiterations)
    {
        const bool valid = m_integ.reiterative_forward(m_elapsed, timestep, *this, ode, reiterations);
        register_forces_into_entities();
        reset_forces();
        retrieve();
        return valid;
    }
    bool engine2D::embedded_forward(float &timestep)
    {
        const bool valid = m_integ.embedded_forward(m_elapsed, timestep, *this, ode);
        register_forces_into_entities();
        reset_forces();
        retrieve();
        return valid;
    }

    void engine2D::load_velocities_and_added_forces(std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        for (std::size_t i = 0; i < m_entities.size(); i++)
        {
            const std::size_t index = 6 * i;
            const alg::vec2 &vel = m_entities[i].vel();
            const float angvel = m_entities[i].angvel();
            stchanges[index] = vel.x;
            stchanges[index + 1] = vel.y;
            stchanges[index + 2] = angvel;
            if (m_entities[i].kinematic())
            {
                const alg::vec2 &force = m_entities[i].added_force();
                const float torque = m_entities[i].added_torque();
                load_force(stchanges, force, torque, index);
            }
        }
    }

    void engine2D::register_forces_into_entities() // TODO: Change name register to load wtf
    {
        const std::vector<float> step = m_integ.state().step();
        for (std::size_t i = 0; i < m_entities.size(); i++)
        {
            const std::size_t index = 6 * i;
            m_entities[i].m_force = {step[index + 3], step[index + 4]};
            m_entities[i].m_torque = step[index + 5];
        }
    }

    void engine2D::load_force(std::vector<float> &stchanges,
                              const alg::vec2 &force,
                              float torque,
                              std::size_t index)
    {
        stchanges[index + 3] += force.x;
        stchanges[index + 4] += force.y;
        stchanges[index + 5] += torque;
    }

    void engine2D::validate()
    {
        m_collider.validate();
        m_compeller.validate();
        for (const std::shared_ptr<force2D> &f : m_forces)
            f->validate();
        for (const std::shared_ptr<interaction2D> &i : m_interactions)
            i->validate();
        for (auto it = m_springs.begin(); it != m_springs.end();)
            if (!it->try_validate())
            {
                m_callbacks.spring_removal(*it);
                it = m_springs.erase(it);
            }
            else
                ++it;
    }

    void engine2D::load_interactions_and_externals(std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        for (const std::shared_ptr<const force2D> f : m_forces)
            for (const_entity2D_ptr e : f->entities())
            {
                if (!e->kinematic())
                    continue;
                const std::size_t index = 6 * e.index();
                const auto [force, torque] = f->force(*e);
                load_force(stchanges, force, torque, index);
            }
        for (const spring2D &s : m_springs)
        {
            const std::size_t index1 = 6 * s.e1().index(),
                              index2 = 6 * s.e2().index();
            const auto [force, t1, t2] = s.force();
            if (s.e1()->kinematic())
                load_force(stchanges, force, t1, index1);
            if (s.e2()->kinematic())
                load_force(stchanges, -force, t2, index2);
        }
        for (const std::shared_ptr<const interaction2D> i : m_interactions)
            for (const_entity2D_ptr e1 : i->entities())
            {
                if (!e1->kinematic())
                    continue;
                const std::size_t index = 6 * e1.index();
                for (const_entity2D_ptr e2 : i->entities())
                    if (e1 != e2)
                    {
                        const auto [force, torque] = i->force(*e1, *e2);
                        load_force(stchanges, force, torque, index);
                    }
            }
    }

    std::vector<float> engine2D::inverse_masses() const
    {
        PERF_FUNCTION()
        std::vector<float> inv_masses;
        inv_masses.reserve(3 * m_entities.size());
        for (std::size_t i = 0; i < m_entities.size(); i++)
        {
            const float inv_mass = 1.f / m_entities[i].mass(),
                        inv_inertia = 1.f / m_entities[i].inertia();
            inv_masses.insert(inv_masses.end(), {inv_mass, inv_mass, inv_inertia});
        }
        return inv_masses;
    }

    void engine2D::reset_forces()
    {
        for (entity2D &e : m_entities)
        {
            e.m_added_force = alg::vec2::zero;
            e.m_added_torque = 0.f;
        }
    }

    entity2D_ptr engine2D::add_entity(const alg::vec2 &pos,
                                      const alg::vec2 &vel,
                                      const float angpos,
                                      const float angvel,
                                      const float mass,
                                      const float charge,
                                      const std::vector<alg::vec2> &vertices,
                                      const bool kinematic)
    {
        entity2D &e = m_entities.emplace_back(pos, vel, angpos, angvel, mass, charge, vertices, kinematic);
        const entity2D_ptr e_ptr = {&m_entities, m_entities.size() - 1};

        rk::state &state = m_integ.state();
        e.m_index = m_entities.size() - 1;
        e.m_state = &state;

        state.append({pos.x, pos.y, angpos,
                      vel.x, vel.y, angvel});
        m_collider.add_entity_intervals(e_ptr);
        e.retrieve();

        DBG_LOG("Added entity with index %zu and id %zu.\n", e.m_index, e.m_id)
#ifdef DEBUG
        for (std::size_t i = 0; i < m_entities.size() - 1; i++)
            DBG_ASSERT(m_entities[i].m_id != e.m_id, "Added entity has the same id as entity with index %zu.\n", i)
#endif
        m_callbacks.entity_addition(e_ptr);
        return e_ptr;
    }

    bool engine2D::remove_entity(const std::size_t index)
    {
        if (index >= m_entities.size())
        {
            DBG_LOG("Index exceeds entity array bounds. Aborting... - index: %zu, size: %zu\n", index, m_entities.size())
            return false;
        }

        m_callbacks.early_entity_removal(m_entities[index]);
        rk::state &state = m_integ.state();
        if (index == m_entities.size() - 1)
            m_entities.pop_back();
        else
        {
            m_entities[index] = m_entities.back();
            m_entities.pop_back();
            m_entities[index].m_index = index;
            m_entities[index].m_state = &state;
        }

        for (std::size_t i = 0; i < 6; i++)
            state[6 * index + i] = state[state.size() - 6 + i];
        state.resize(6 * m_entities.size());

        validate();
        m_collider.update_quad_tree();
        m_callbacks.late_entity_removal(index);
        return true;
    }

    bool engine2D::remove_entity(const entity2D &e) { return remove_entity(e.index()); }

    void engine2D::add_force(const std::shared_ptr<force2D> &force) { m_forces.emplace_back(force); }
    void engine2D::add_interaction(const std::shared_ptr<interaction2D> &inter) { m_interactions.emplace_back(inter); }

    spring2D &engine2D::add_spring(entity2D_ptr e1,
                                   entity2D_ptr e2,
                                   const float stiffness,
                                   const float dampening,
                                   const float length)
    {
        spring2D &sp = m_springs.emplace_back(e1, e2, stiffness, dampening, length);
        m_callbacks.spring_addition(&sp);
        return sp;
    }

    spring2D &engine2D::add_spring(entity2D_ptr e1,
                                   entity2D_ptr e2,
                                   const alg::vec2 &joint1,
                                   const alg::vec2 &joint2,
                                   const float stiffness,
                                   const float dampening,
                                   const float length)
    {
        spring2D &sp = m_springs.emplace_back(e1, e2, joint1, joint2, stiffness, dampening, length);
        m_callbacks.spring_addition(&sp);
        return sp;
    }

    void engine2D::add_constraint(const std::shared_ptr<constraint_interface2D> &ctr) { m_compeller.add_constraint(ctr); }
    bool engine2D::remove_constraint(const std::shared_ptr<const constraint_interface2D> &ctr) { return m_compeller.remove_constraint(ctr); }

    bool engine2D::remove_force(const std::shared_ptr<force2D> &force)
    {
        for (auto it = m_forces.begin(); it != m_forces.end(); ++it)
            if (*it == force)
            {
                m_forces.erase(it);
                return true;
            }
        return false;
    }
    bool engine2D::remove_interaction(const std::shared_ptr<interaction2D> &inter)
    {
        for (auto it = m_interactions.begin(); it != m_interactions.end(); ++it)
            if (*it == inter)
            {
                m_interactions.erase(it);
                return true;
            }
        return false;
    }
    bool engine2D::remove_spring(std::size_t index)
    {
        if (index >= m_springs.size())
        {
            DBG_LOG("Array index out of bounds. Aborting... - index: %zu, size: %zu\n", index, m_springs.size())
            return false;
        }
        m_callbacks.spring_removal(m_springs[index]);
        m_springs.erase(m_springs.begin() + index);
        return true;
    }
    bool engine2D::remove_spring(const spring2D &sp)
    {
        for (std::size_t i = 0; i < m_springs.size(); i++)
            if (sp.e1() == m_springs[i].e1() && sp.e2() == m_springs[i].e2())
                return remove_spring(i);
        return false;
    }

    void engine2D::clear_entities()
    {
        for (std::size_t i = m_entities.size() - 1; i < m_entities.size(); i--)
            remove_entity(i);
    }
    void engine2D::clear_forces() { m_forces.clear(); }
    void engine2D::clear_interactions() { m_interactions.clear(); }
    void engine2D::clear_springs() { m_springs.clear(); }
    void engine2D::clear_constraints() { m_compeller.clear_constraints(); }
    void engine2D::clear()
    {
        m_forces.clear();
        m_interactions.clear();
        m_springs.clear();
        m_compeller.clear_constraints();
        clear_entities();
    }

    void engine2D::write(ini::output &out) const
    {
        out.write("elapsed", m_elapsed);
        out.begin_section("tableau");
        m_integ.tableau().write(out);
        out.end_section();
        out.begin_section("collider");
        m_collider.write(out);
        out.end_section();

        std::string section = "entity";
        for (const entity2D &e : m_entities)
        {
            out.begin_section(section + std::to_string(e.index()));
            e.write(out);
            out.end_section();
        }

        section = "spring";
        std::size_t index = 0;
        for (const spring2D &sp : m_springs)
        {
            out.begin_section(section + std::to_string(index++));
            sp.write(out);
            out.end_section();
        }

        section = "rigid_bar";
        index = 0;
        for (const auto &ctr : m_compeller.constraints())
        {
            const auto rb = std::dynamic_pointer_cast<const rigid_bar2D>(ctr);
            if (!rb)
                continue;
            out.begin_section(section + std::to_string(index++));
            rb->write(out);
            out.end_section();
        }
    }

    void engine2D::read(ini::input &in)
    {
        clear_entities();
        m_elapsed = in.readf("elapsed");
        in.begin_section("tableau");
        rk::butcher_tableau tb;
        tb.read(in);
        m_integ.tableau(tb);
        in.end_section();
        in.begin_section("collider");
        m_collider.read(in);
        in.end_section();

        std::string section = "entity";
        std::size_t index = 0;
        while (true)
        {
            in.begin_section(section + std::to_string(index++));
            if (!in.contains_section())
            {
                in.end_section();
                break;
            }
            add_entity()->read(in);
            in.end_section();
        }

        section = "spring";
        index = 0;
        while (true)
        {
            in.begin_section(section + std::to_string(index++));
            if (!in.contains_section())
            {
                in.end_section();
                break;
            }
            const bool has_joints = (bool)in.readi("has_joints");
            const std::size_t idx1 = in.readi("e1"), idx2 = in.readi("e2");
            const entity2D_ptr e1 = (*this)[idx1], e2 = (*this)[idx2];

            if (has_joints)
            {
                const alg::vec2 joint1 = {in.readf("joint1x"), in.readf("joint1y")},
                                joint2 = {in.readf("joint2x"), in.readf("joint2y")};
                add_spring(e1, e2, joint1, joint2).read(in);
            }
            else
                add_spring(e1, e2).read(in);
            in.end_section();
        }

        section = "rigid_bar";
        index = 0;
        while (true)
        {
            in.begin_section(section + std::to_string(index++));
            if (!in.contains_section())
            {
                in.end_section();
                break;
            }
            const bool has_joints = (bool)in.readi("has_joints");
            const std::size_t idx1 = in.readi("e1"), idx2 = in.readi("e2");
            const entity2D_ptr e1 = (*this)[idx1], e2 = (*this)[idx2];

            if (has_joints)
            {
                const alg::vec2 joint1 = {in.readf("joint1x"), in.readf("joint1y")},
                                joint2 = {in.readf("joint2x"), in.readf("joint2y")};
                const auto rb = std::make_shared<rigid_bar2D>(e1, e2, joint1, joint2);
                rb->read(in);
                m_compeller.add_constraint(rb);
            }
            else
            {
                const auto rb = std::make_shared<rigid_bar2D>(e1, e2);
                rb->read(in);
                m_compeller.add_constraint(rb);
            }
            in.end_section();
        }
    }

    void engine2D::checkpoint() { m_checkpoint = std::make_tuple(m_elapsed, m_integ.state().vars(), m_entities); }
    void engine2D::revert()
    {
        const auto &[elapsed, vars, entities] = m_checkpoint;
        DBG_ASSERT(m_integ.state().vars().size() == vars.size() &&
                       m_entities.size() == entities.size(),
                   "Cannot revert to a checkpoint where the number of entities differ. Entities now: %zu, entities before: %zu.\n", m_entities.size(), entities.size())

        m_elapsed = elapsed;
        m_integ.state().vars(vars);
        m_entities = entities;
    }

    float engine2D::kinetic_energy() const
    {
        float ke = 0.f;
        for (const entity2D &e : m_entities)
            ke += e.kinetic_energy();
        return ke;
    }
    float engine2D::potential_energy() const
    {
        float pot = 0.f;
        for (const auto &force : m_forces)
            pot += force->potential_energy();
        for (const auto &inter : m_interactions)
            pot += inter->potential_energy();
        for (const spring2D &sp : m_springs)
            pot += sp.potential_energy();
        return pot;
    }
    float engine2D::energy() const { return kinetic_energy() + potential_energy(); }

    const_entity2D_ptr engine2D::entity_from_index(const std::size_t index) const
    {
        DBG_ASSERT(index < m_entities.size(), "Index exceeds array bounds - index: %zu, size: %zu.\n", index, m_entities.size())
        return {&m_entities, index};
    }
    entity2D_ptr engine2D::entity_from_index(const std::size_t index)
    {
        DBG_ASSERT(index < m_entities.size(), "Index exceeds array bounds - index: %zu, size: %zu.\n", index, m_entities.size())
        return {&m_entities, index};
    }

    const_entity2D_ptr engine2D::entity_from_id(const std::size_t id) const
    {
        for (std::size_t i = 0; i < m_entities.size(); i++)
            if (m_entities[i].id() == id)
                return {&m_entities, i};
        return nullptr;
    }
    entity2D_ptr engine2D::entity_from_id(const std::size_t id)
    {
        for (std::size_t i = 0; i < m_entities.size(); i++)
            if (m_entities[i].id() == id)
                return {&m_entities, i};
        return nullptr;
    }

    const spring2D *engine2D::spring_from_entities(const entity2D &e1,
                                                   const entity2D &e2) const
    {
        for (const spring2D &sp : m_springs)
            if ((*sp.e1() == e1 && *sp.e2() == e2) ||
                (*sp.e1() == e2 && *sp.e2() == e1))
                return &sp;
        return nullptr;
    }
    spring2D *engine2D::spring_from_entities(const entity2D &e1,
                                             const entity2D &e2)
    {
        for (spring2D &sp : m_springs)
            if ((*sp.e1() == e1 && *sp.e2() == e2) ||
                (*sp.e1() == e2 && *sp.e2() == e1))
                return &sp;
        return nullptr;
    }

    std::shared_ptr<const rigid_bar2D> engine2D::rbar_from_entities(const entity2D &e1,
                                                                    const entity2D &e2) const
    {
        for (const auto &ctr : m_compeller.constraints())
        {
            const auto rb = std::dynamic_pointer_cast<const rigid_bar2D>(ctr);
            if (!rb)
                continue;
            if ((*rb->e1() == e1 && *rb->e2() == e2) ||
                (*rb->e1() == e2 && *rb->e2() == e1))
                return rb;
        }
        return nullptr;
    }
    std::shared_ptr<rigid_bar2D> engine2D::rbar_from_entities(const entity2D &e1,
                                                              const entity2D &e2)
    {
        for (const auto &ctr : m_compeller.constraints())
        {
            const auto rb = std::dynamic_pointer_cast<rigid_bar2D>(ctr);
            if (!rb)
                continue;
            if ((*rb->e1() == e1 && *rb->e2() == e2) ||
                (*rb->e1() == e2 && *rb->e2() == e1))
                return rb;
        }
        return nullptr;
    }

    const_entity2D_ptr engine2D::operator[](const std::size_t index) const { return entity_from_index(index); }
    entity2D_ptr engine2D::operator[](const std::size_t index) { return entity_from_index(index); }

    std::vector<const_entity2D_ptr> engine2D::operator[](const geo::aabb2D &aabb) const
    {
        std::vector<const_entity2D_ptr> in_area;
        in_area.reserve(m_entities.size() / 2);
        for (const entity2D &e : m_entities)
            if (e.aabb().overlaps(aabb))
                in_area.emplace_back(&m_entities, e.index());
        return in_area;
    }
    std::vector<entity2D_ptr> engine2D::operator[](const geo::aabb2D &aabb)
    {
        std::vector<entity2D_ptr> in_area;
        in_area.reserve(m_entities.size() / 2);
        for (const entity2D &e : m_entities)
            if (e.aabb().overlaps(aabb))
                in_area.emplace_back(&m_entities, e.index());
        return in_area;
    }

    const std::vector<std::shared_ptr<force2D>> &engine2D::forces() const { return m_forces; }
    const std::vector<std::shared_ptr<interaction2D>> &engine2D::interactions() const { return m_interactions; }
    const std::vector<spring2D> &engine2D::springs() const { return m_springs; }

    utils::vector_view<std::shared_ptr<force2D>> engine2D::forces() { return m_forces; }
    utils::vector_view<std::shared_ptr<interaction2D>> engine2D::interactions() { return m_interactions; }
    utils::vector_view<spring2D> engine2D::springs() { return m_springs; }

    const_entity2D_ptr engine2D::operator[](const alg::vec2 &point) const
    {
        const geo::aabb2D aabb = point;
        for (const entity2D &e : m_entities)
            if (e.aabb().overlaps(aabb))
                return {&m_entities, e.index()};
        return nullptr;
    }
    entity2D_ptr engine2D::operator[](const alg::vec2 &point)
    {
        const geo::aabb2D aabb = point;
        for (const entity2D &e : m_entities)
            if (e.aabb().overlaps(aabb))
                return {&m_entities, e.index()};
        return nullptr;
    }

    const std::vector<entity2D> &engine2D::entities() const { return m_entities; }
    utils::vector_view<entity2D> engine2D::entities() { return m_entities; }
    std::size_t engine2D::size() const { return m_entities.size(); }

    const rk::integrator &engine2D::integrator() const { return m_integ; }
    rk::integrator &engine2D::integrator() { return m_integ; }

    const collider2D &engine2D::collider() const { return m_collider; }
    collider2D &engine2D::collider() { return m_collider; }

    const compeller2D &engine2D::compeller() const { return m_compeller; }

    const callbacks &engine2D::callbacks() const { return m_callbacks; }
    callbacks &engine2D::callbacks() { return m_callbacks; }

    float engine2D::elapsed() const { return m_elapsed; }
}