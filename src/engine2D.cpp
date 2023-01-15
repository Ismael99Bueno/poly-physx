#include "engine2D.hpp"
#include "ode2D.hpp"
#include "perf.hpp"
#include <random>

namespace phys
{
    engine2D::engine2D(const rk::butcher_tableau &table,
                       const std::size_t allocations) : m_integ(table, m_state),
                                                        m_collider(m_entities, 2 * allocations),
                                                        m_compeller(m_entities, allocations)
    {
        m_entities.reserve(allocations);
        m_state.reserve(6 * allocations);
        m_integ.reserve(6 * allocations);
    }

    void engine2D::retrieve(const std::vector<float> &state)
    {
        PERF_FUNCTION()
        for (std::size_t i = 0; i < m_entities.size(); i++)
            m_entities[i].retrieve(utils::const_vec_ptr(&state, 6 * i));
    }

    void engine2D::retrieve() { retrieve(m_state); }

    bool engine2D::raw_forward(float &dt)
    {
        const bool valid = m_integ.raw_forward(m_t, dt, *this, ode);
        register_forces_onto_entities();
        reset_forces();
        return valid;
    }
    bool engine2D::reiterative_forward(float &dt, const std::size_t reiterations)
    {
        const bool valid = m_integ.reiterative_forward(m_t, dt, *this, ode, reiterations);
        register_forces_onto_entities();
        reset_forces();
        return valid;
    }
    bool engine2D::embedded_forward(float &dt)
    {
        const bool valid = m_integ.embedded_forward(m_t, dt, *this, ode);
        register_forces_onto_entities();
        reset_forces();
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
            if (m_entities[i].dynamic())
            {
                const alg::vec2 &force = m_entities[i].added_force();
                const float torque = m_entities[i].added_torque();
                load_force(stchanges, force, torque, index);
            }
        }
    }

    void engine2D::register_forces_onto_entities()
    {
        const std::vector<float> step = m_integ.step();
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

    void engine2D::load_interactions_and_externals(std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        for (const std::shared_ptr<const force2D> &f : m_forces)
            for (const const_entity2D_ptr &e : f->entities())
            {
                if (!e->dynamic())
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
            if (s.e1()->dynamic())
                load_force(stchanges, force, t1, index1);
            if (s.e2()->dynamic())
                load_force(stchanges, -force, t2, index2);
        }
        for (const std::shared_ptr<const interaction2D> &i : m_inters)
            for (const const_entity2D_ptr &e1 : i->entities())
            {
                if (!e1->dynamic())
                    continue;
                const std::size_t index = 6 * e1.index();
                for (const const_entity2D_ptr &e2 : i->entities())
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
            e.m_added_force = {0.f, 0.f};
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
                                      const bool dynamic)
    {
        entity2D &e = m_entities.emplace_back(pos, vel, angpos, angvel, mass, charge, vertices, dynamic);
        const entity2D_ptr e_ptr = {&m_entities, m_entities.size() - 1};

        e.m_index = m_entities.size() - 1;
        e.m_buffer = utils::vec_ptr(&m_state, m_state.size());

        m_state.insert(m_state.end(), {pos.x, pos.y, angpos,
                                       vel.x, vel.y, angvel});
        m_collider.add_entity_intervals(e_ptr);

        e.retrieve();
        m_integ.resize();
        DBG_LOG("Added entity with index %zu and id %zu.\n", e.m_index, e.m_id)
#ifdef DEBUG
        for (std::size_t i = 0; i < m_entities.size() - 1; i++)
            DBG_ASSERT(m_entities[i].m_id != e.m_id, "Added entity has the same id as entity with index %zu.\n", i)
#endif
        for (const add_callback &cb : m_on_entity_addition)
            cb(e_ptr);
        return e_ptr;
    }

    void engine2D::remove_entity(const std::size_t index)
    {
        DBG_ASSERT(index < m_entities.size(), "Index exceeds entity array bounds - index: %zu, size: %zu\n", index, m_entities.size())
        if (index == m_entities.size() - 1)
            m_entities.pop_back();
        else
        {
            m_entities[index] = m_entities.back();
            m_entities.pop_back();
            m_entities[index].m_index = index;
            m_entities[index].m_buffer = utils::vec_ptr(&m_state, 6 * index);
        }

        for (std::size_t i = 0; i < 6; i++)
            m_state[6 * index + i] = m_state[m_state.size() - 6 + i];
        m_state.resize(6 * m_entities.size());
        m_collider.validate();
        m_compeller.validate();
        for (const std::shared_ptr<force2D> &f : m_forces)
            f->validate();
        for (const std::shared_ptr<interaction2D> &i : m_inters)
            i->validate();
        for (auto it = m_springs.begin(); it != m_springs.end();)
            if (!it->try_validate())
                it = m_springs.erase(it);
            else
                ++it;

        m_integ.resize();
        m_collider.update_quad_tree();
        for (const remove_callback &cb : m_on_entity_removal)
            cb(index);
    }

    void engine2D::remove_entity(const const_entity2D_ptr &e) { remove_entity(e.index()); }

    void engine2D::add_force(const std::shared_ptr<force2D> &force) { m_forces.emplace_back(force); }
    void engine2D::add_interaction(const std::shared_ptr<interaction2D> &inter) { m_inters.emplace_back(inter); }
    void engine2D::add_spring(const spring2D &spring) { m_springs.emplace_back(spring); }

    void engine2D::remove_force(const std::shared_ptr<force2D> &force)
    {
        m_forces.erase(std::remove(m_forces.begin(), m_forces.end(), force), m_forces.end());
    }
    void engine2D::remove_interaction(const std::shared_ptr<interaction2D> &inter)
    {
        m_inters.erase(std::remove(m_inters.begin(), m_inters.end(), inter), m_inters.end());
    }
    void engine2D::remove_spring(std::size_t index)
    {
        DBG_LOG_IF(index >= m_springs.size(), "Failed to remove spring, index outside array bounds! - index: %zu, size: %zu\n", index, m_springs.size())
        if (index >= m_springs.size())
            return;
        m_springs.erase(m_springs.begin() + index);
    }

    void engine2D::clear_entities()
    {
        for (std::size_t i = m_entities.size() - 1; i >= 0 && i < m_entities.size(); i--)
            remove_entity(i);
    }
    void engine2D::clear_forces() { m_forces.clear(); }
    void engine2D::clear_interactions() { m_inters.clear(); }
    void engine2D::clear_springs() { m_springs.clear(); }
    void engine2D::clear()
    {
        m_forces.clear();
        m_inters.clear();
        m_springs.clear();
        m_compeller.clear_constraints();
        clear_entities();
    }

    void engine2D::on_entity_addition(const add_callback &on_add) { m_on_entity_addition.emplace_back(on_add); }
    void engine2D::on_entity_removal(const remove_callback &on_remove) { m_on_entity_removal.emplace_back(on_remove); }

    const_entity2D_ptr engine2D::operator[](std::size_t index) const { return {&m_entities, index}; }
    entity2D_ptr engine2D::operator[](std::size_t index) { return {&m_entities, index}; }

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
    const std::vector<std::shared_ptr<interaction2D>> &engine2D::interactions() const { return m_inters; }
    const std::vector<spring2D> &engine2D::springs() const { return m_springs; }

    std::vector<std::shared_ptr<force2D>> &engine2D::forces() { return m_forces; }
    std::vector<std::shared_ptr<interaction2D>> &engine2D::interactions() { return m_inters; }
    std::vector<spring2D> &engine2D::springs() { return m_springs; }

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
    std::vector<entity2D> &engine2D::entities() { return m_entities; }
    std::size_t engine2D::size() const { return m_entities.size(); }

    const rk::integrator &engine2D::integrator() const { return m_integ; }
    rk::integrator &engine2D::integrator() { return m_integ; }

    const std::vector<float> &engine2D::state() const { return m_state; }

    const compeller2D &engine2D::compeller() const { return m_compeller; }
    compeller2D &engine2D::compeller() { return m_compeller; }

    const collider2D &engine2D::collider() const { return m_collider; }
    collider2D &engine2D::collider() { return m_collider; }

    float engine2D::elapsed() const { return m_t; }
}