#include "ppx/pch.hpp"
#include "ppx/engine2D.hpp"
#include "ppx/ode2D.hpp"
#include "perf/perf.hpp"
#include "geo/intersection.hpp"

namespace ppx
{
    engine2D::engine2D(const rk::butcher_tableau &table,
                       const std::size_t allocations) : m_collider(engine_key(), &m_entities, 2 * allocations),
                                                        m_compeller(engine_key(), &m_entities, allocations, &m_events),
                                                        m_integ(table), m_events(engine_key())
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
        reset_entities();
        retrieve();
        return valid;
    }
    bool engine2D::reiterative_forward(float &timestep, const std::uint8_t reiterations)
    {
        const bool valid = m_integ.reiterative_forward(m_elapsed, timestep, *this, ode, reiterations);
        register_forces_into_entities();
        reset_entities();
        retrieve();
        return valid;
    }
    bool engine2D::embedded_forward(float &timestep)
    {
        const bool valid = m_integ.embedded_forward(m_elapsed, timestep, *this, ode);
        register_forces_into_entities();
        reset_entities();
        retrieve();
        return valid;
    }

    static void load_force(std::vector<float> &stchanges,
                           const glm::vec2 &force,
                           float torque,
                           std::size_t index)
    {
        stchanges[index + 3] += force.x;
        stchanges[index + 4] += force.y;
        stchanges[index + 5] += torque;
    }

    void engine2D::load_velocities_and_added_forces(std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        for (std::size_t i = 0; i < m_entities.size(); i++)
        {
            const std::size_t index = 6 * i;
            const glm::vec2 &vel = m_entities[i].vel();
            const float angvel = m_entities[i].angvel();
            stchanges[index] = vel.x;
            stchanges[index + 1] = vel.y;
            stchanges[index + 2] = angvel;
            if (m_entities[i].kinematic())
            {
                const glm::vec2 &force = m_entities[i].added_force();
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
                m_events.on_spring_removal(*it);
                it = m_springs.erase(it);
            }
            else
                ++it;
    }

    void engine2D::load_interactions_and_externals(std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        for (const std::shared_ptr<force2D> &f : m_forces)
            for (const const_entity2D_ptr &e : f->entities())
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
        for (const std::shared_ptr<interaction2D> &i : m_interactions)
            for (const const_entity2D_ptr &e1 : i->entities())
            {
                if (!e1->kinematic())
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
            const float inv_mass = m_entities[i].kinematic() ? (1.f / m_entities[i].mass()) : 0.f,
                        inv_inertia = m_entities[i].kinematic() ? (1.f / m_entities[i].inertia()) : 0.f;
            inv_masses.insert(inv_masses.end(), {inv_mass, inv_mass, inv_inertia});
        }
        return inv_masses;
    }

    void engine2D::reset_entities()
    {
        for (entity2D &e : m_entities)
        {
            e.m_added_force = glm::vec2(0.f);
            e.m_added_torque = 0.f;
            e.events().reset();
        }
    }

    entity2D_ptr engine2D::process_entity_addition(entity2D &e)
    {
        rk::state &state = m_integ.state();
        e.m_index = m_entities.size() - 1;
        e.m_state = &state;

        const entity2D_ptr e_ptr = {&m_entities, m_entities.size() - 1};
        const glm::vec2 &pos = e.pos(), &vel = e.vel();
        state.append({pos.x, pos.y, e.angpos(),
                      vel.x, vel.y, e.angvel()});
        e.retrieve();
        m_collider.add_entity_intervals(e_ptr);

        DBG_LOG("Added entity with index %zu and id %zu.\n", e.m_index, e.m_id)
#ifdef DEBUG
        for (std::size_t i = 0; i < m_entities.size() - 1; i++)
            DBG_ASSERT(m_entities[i].m_id != e.m_id, "Added entity has the same id as entity with index %zu.\n", i)
#endif
        m_events.on_entity_addition(e_ptr);
        return e_ptr;
    }

    bool engine2D::remove_entity(std::size_t index)
    {
        if (index >= m_entities.size())
        {
            DBG_LOG("Index exceeds entity array bounds. Aborting... - index: %zu, size: %zu\n", index, m_entities.size())
            return false;
        }

        m_events.on_early_entity_removal(m_entities[index]);
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
        m_events.on_late_entity_removal(std::move(index)); // It just made me do this...
        return true;
    }

    bool engine2D::remove_entity(const entity2D &e) { return remove_entity(e.index()); }
    bool engine2D::remove_force(const std::shared_ptr<const force2D> &force)
    {
        for (auto it = m_forces.begin(); it != m_forces.end(); ++it)
            if (*it == force)
            {
                m_events.on_force_removal(*it);
                m_forces.erase(it);
                return true;
            }
        return false;
    }
    bool engine2D::remove_interaction(const std::shared_ptr<const interaction2D> &inter)
    {
        for (auto it = m_interactions.begin(); it != m_interactions.end(); ++it)
            if (*it == inter)
            {
                m_events.on_interaction_removal(*it);
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
        m_events.on_spring_removal(m_springs[index]);
        m_springs.erase(m_springs.begin() + (long)index);
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

    std::optional<std::size_t> engine2D::index_from_id(const uuid id) const
    {
        for (std::size_t i = 0; i < m_entities.size(); i++)
            if (m_entities[i].id() == id)
                return i;
        return {};
    }

    const_entity2D_ptr engine2D::from_id(uuid id) const
    {
        const auto index = index_from_id(id);
        return index ? (*this)[index.value()] : nullptr;
    }

    entity2D_ptr engine2D::from_id(uuid id)
    {
        const auto index = index_from_id(id);
        return index ? (*this)[index.value()] : nullptr;
    }

    const_entity2D_ptr engine2D::operator[](const std::size_t index) const
    {
        DBG_ASSERT(index < m_entities.size(), "Index exceeds array bounds - index: %zu, size: %zu.\n", index, m_entities.size())
        return {&m_entities, index};
    }
    entity2D_ptr engine2D::operator[](const std::size_t index)
    {
        DBG_ASSERT(index < m_entities.size(), "Index exceeds array bounds - index: %zu, size: %zu.\n", index, m_entities.size())
        return {&m_entities, index};
    }

    std::vector<const_entity2D_ptr> engine2D::operator[](const geo::aabb2D &aabb) const
    {
        std::vector<const_entity2D_ptr> in_area;
        in_area.reserve(m_entities.size() / 2);

        for (const entity2D &e : m_entities)
            if (geo::intersect(e.shape().bounding_box(), aabb))
                in_area.emplace_back(&m_entities, e.index());
        return in_area;
    }
    std::vector<entity2D_ptr> engine2D::operator[](const geo::aabb2D &aabb)
    {
        std::vector<entity2D_ptr> in_area;
        in_area.reserve(m_entities.size() / 2);
        for (const entity2D &e : m_entities)
            if (geo::intersect(e.shape().bounding_box(), aabb))
                in_area.emplace_back(&m_entities, e.index());
        return in_area;
    }

    const std::vector<std::shared_ptr<force2D>> &engine2D::forces() const { return m_forces; }
    const std::vector<std::shared_ptr<interaction2D>> &engine2D::interactions() const { return m_interactions; }
    const std::vector<spring2D> &engine2D::springs() const { return m_springs; }

    cvw::vector<std::shared_ptr<force2D>> engine2D::forces() { return m_forces; }
    cvw::vector<std::shared_ptr<interaction2D>> engine2D::interactions() { return m_interactions; }
    cvw::vector<spring2D> engine2D::springs() { return m_springs; }

    const_entity2D_ptr engine2D::operator[](const glm::vec2 &point) const
    {
        const geo::aabb2D aabb = point;
        for (const entity2D &e : m_entities)
            if (geo::intersect(e.shape().bounding_box(), aabb))
                return {&m_entities, e.index()};
        return nullptr;
    }
    entity2D_ptr engine2D::operator[](const glm::vec2 &point)
    {
        const geo::aabb2D aabb = point;
        for (const entity2D &e : m_entities)
            if (geo::intersect(e.shape().bounding_box(), aabb))
                return {&m_entities, e.index()};
        return nullptr;
    }

    const std::vector<entity2D> &engine2D::entities() const { return m_entities; }
    cvw::vector<entity2D> engine2D::entities() { return m_entities; }
    std::size_t engine2D::size() const { return m_entities.size(); }

    const rk::integrator &engine2D::integrator() const { return m_integ; }
    rk::integrator &engine2D::integrator() { return m_integ; }

    const collider2D &engine2D::collider() const { return m_collider; }
    collider2D &engine2D::collider() { return m_collider; }

    const compeller2D &engine2D::compeller() const { return m_compeller; }
    compeller2D &engine2D::compeller() { return m_compeller; }

    engine_events &engine2D::events() { return m_events; }

    float engine2D::elapsed() const { return m_elapsed; }
#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const engine2D &eng)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "Entities" << YAML::Value << eng.entities();
        out << YAML::Key << "Collider" << YAML::Value << eng.collider();
        out << YAML::Key << "Springs" << YAML::Value << eng.springs();
        out << YAML::Key << "Rigid bars" << YAML::Value << YAML::BeginSeq;
        for (const auto &ctr : eng.compeller().constraints())
        {
            const auto rb = std::dynamic_pointer_cast<rigid_bar2D>(ctr);
            if (rb)
                out << *rb;
        }
        out << YAML::EndSeq;
        out << YAML::Key << "Forces" << YAML::Value << YAML::BeginMap;
        for (const auto &f : eng.forces())
        {
            out << YAML::Key << f->name() << YAML::Value << YAML::Flow << YAML::BeginSeq;
            for (const auto &e : f->entities())
                out << e.index();
            out << YAML::EndSeq;
        }
        out << YAML::EndMap;
        out << YAML::Key << "Interactions" << YAML::Value << YAML::BeginMap;
        for (const auto &i : eng.interactions())
        {
            out << YAML::Key << i->name() << YAML::Value << YAML::Flow << YAML::BeginSeq;
            for (const auto &e : i->entities())
                out << e.index();
            out << YAML::EndSeq;
        }
        out << YAML::EndMap;
        // Save checkpoint?
        out << YAML::Key << "Integrator" << YAML::Value << eng.integrator();
        out << YAML::Key << "Elapsed" << YAML::Value << eng.elapsed();
        out << YAML::EndMap;
        return out;
    }
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    Node convert<ppx::engine2D>::encode(const ppx::engine2D &eng)
    {
        Node node;
        node["Entities"] = eng.entities();
        node["Collider"] = eng.collider();
        node["Springs"] = eng.springs();
        for (const auto &ctr : eng.compeller().constraints())
        {
            const auto rb = std::dynamic_pointer_cast<ppx::rigid_bar2D>(ctr);
            if (rb)
                node["Rigid bars"].push_back(*rb);
        }
        for (const auto &f : eng.forces())
        {
            for (const auto &e : f->entities())
                node["Forces"][f->name()].push_back(e.index());
            node["Forces"][f->name()].SetStyle(YAML::EmitterStyle::Flow);
        }
        for (const auto &i : eng.interactions())
        {
            for (const auto &e : i->entities())
                node["Interactions"][i->name()].push_back(e.index());
            node["Interactions"][i->name()].SetStyle(YAML::EmitterStyle::Flow);
        }
        // Save checkpoint?
        node["Integrator"] = eng.integrator();
        node["Elapsed"] = eng.elapsed();
        return node;
    }
    bool convert<ppx::engine2D>::decode(const Node &node, ppx::engine2D &eng)
    {
        if (!node.IsMap() || node.size() != 8)
            return false;

        eng.clear_entities();
        eng.m_integ = node["Integrator"].as<rk::integrator>();
        eng.m_integ.state().clear();

        for (const Node &n : node["Entities"])
            eng.add_entity(n.as<ppx::entity2D>());

        node["Collider"].as<ppx::collider2D>(eng.collider());
        for (const Node &n : node["Springs"])
        {
            const std::size_t idx1 = n["Index1"].as<std::size_t>(),
                              idx2 = n["Index2"].as<std::size_t>();
            if (n["Anchor1"])
            {
                ppx::spring2D &sp = eng.add_spring(eng[idx1], eng[idx2],
                                                   n["Anchor1"].as<glm::vec2>(),
                                                   n["Anchor2"].as<glm::vec2>());
                n.as<ppx::spring2D>(sp);
                continue;
            }
            ppx::spring2D &sp = eng.add_spring(eng[idx1], eng[idx2]);
            n.as<ppx::spring2D>(sp);
        }

        for (const Node &n : node["Rigid bars"])
        {
            const std::size_t idx1 = n["Index1"].as<std::size_t>(),
                              idx2 = n["Index2"].as<std::size_t>();
            if (n["Anchor1"])
            {
                const auto rb = eng.compeller().add_constraint<ppx::rigid_bar2D>(eng[idx1], eng[idx2],
                                                                                 n["Anchor1"].as<glm::vec2>(),
                                                                                 n["Anchor2"].as<glm::vec2>());
                n.as<ppx::rigid_bar2D>(*rb);
                continue;
            }
            const auto rb = eng.compeller().add_constraint<ppx::rigid_bar2D>(eng[idx1], eng[idx2]);
            n.as<ppx::rigid_bar2D>(*rb);
        }

        for (auto it = node["Forces"].begin(); it != node["Forces"].end(); ++it)
        {
            const auto force = eng.by_name<ppx::force2D>(it->first.as<std::string>().c_str());
            force->clear();
            for (const Node &n : node["Forces"][force->name()])
                force->include(eng[n.as<std::size_t>()]);
        }
        for (auto it = node["Interactions"].begin(); it != node["Interactions"].end(); ++it)
        {
            const auto inter = eng.by_name<ppx::interaction2D>(it->first.as<std::string>().c_str());
            inter->clear();
            for (const Node &n : node["Interactions"][inter->name()])
                inter->include(eng[n.as<std::size_t>()]);
        }

        eng.m_elapsed = node["Elapsed"].as<float>();
        return true;
    };
}
#endif