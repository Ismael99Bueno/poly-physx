#include "engine2D.hpp"
#include "ode2D.hpp"
#include "perf.hpp"

namespace phys
{
    engine2D::engine2D(const rk::tableau &table,
                       const float dt,
                       const std::size_t allocations) : m_dt(dt),
                                                        m_integ(table, m_state),
                                                        m_collider(m_entities),
                                                        m_compeller(m_entities)
    {
        m_entities.reserve(allocations);
        m_state.reserve(6 * allocations);
        m_integ.reserve(6 * allocations);
    }

    void engine2D::retrieve(const std::vector<float> &state)
    {
        PERF_FUNCTION()
        for (std::size_t i = 0; i < m_entities.size(); i++)
            m_entities[i].retrieve(utils::const_vec_ptr(state, 6 * i));
    }

    void engine2D::retrieve() { retrieve(m_state); }

    bool engine2D::raw_forward()
    {
        PERF_FUNCTION()
        const bool valid = m_integ.raw_forward(m_t, m_dt, *this, ode);
        reset_forces();
        return valid;
    }
    bool engine2D::reiterative_forward(const std::size_t reiterations)
    {
        PERF_FUNCTION()
        const bool valid = m_integ.reiterative_forward(m_t, m_dt, *this, ode, reiterations);
        reset_forces();
        return valid;
    }
    bool engine2D::embedded_forward()
    {
        PERF_FUNCTION()
        const bool valid = m_integ.embedded_forward(m_t, m_dt, *this, ode);
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
            const alg::vec2 &force = m_entities[i].added_force();
            const float torque = m_entities[i].added_torque();
            load_force(stchanges, force, torque, index);
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
        for (const force2D *f : m_forces)
            for (const const_entity_ptr &e : f->entities())
            {
                const std::size_t index = 6 * e.index();
                const auto [force, torque] = f->force(*e);
                load_force(stchanges, force, torque, index);
            }
        for (const spring2D &s : m_springs)
        {
            const std::size_t index1 = 6 * s.e1().index(),
                              index2 = 6 * s.e2().index();
            const auto [force, t1, t2] = s.force();
            load_force(stchanges, force, t1, index1);
            load_force(stchanges, -force, t2, index2);
        }
        for (const interaction2D *i : m_inters)
            for (const const_entity_ptr &e1 : i->entities())
            {
                const std::size_t index = 6 * e1.index();
                for (const const_entity_ptr &e2 : i->entities())
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

    entity_ptr engine2D::add_entity(const body2D &body,
                                    const std::vector<alg::vec2> &vertices)
    {
        entity2D &e = m_entities.emplace_back(body, vertices);
        e.m_buffer = utils::vec_ptr(m_state, m_state.size());
        m_state.insert(m_state.end(), {body.pos().x, body.pos().y, body.angpos(),
                                       body.vel().x, body.vel().y, body.angvel()});
        m_collider.add_entity_intervals({m_entities, m_entities.size() - 1});

        e.retrieve();
        m_integ.resize();
        return entity_ptr(m_entities, m_entities.size() - 1);
    }
    entity_ptr engine2D::add_entity(const alg::vec2 &pos,
                                    const alg::vec2 &vel,
                                    const float angpos,
                                    const float angvel,
                                    const float mass,
                                    const float charge,
                                    const std::vector<alg::vec2> &vertices)
    {
        return add_entity(body2D(pos, vel, angpos, angvel, mass, charge), vertices);
    }

    void engine2D::add_constrain(const constrain_interface &c) { m_compeller.add_constrain(c); }
    void engine2D::add_force(const force2D &force) { m_forces.emplace_back(&force); }
    void engine2D::add_interaction(const interaction2D &inter) { m_inters.emplace_back(&inter); }
    void engine2D::add_spring(const spring2D &spring) { m_springs.emplace_back(spring); }

    const_entity_ptr engine2D::operator[](std::size_t index) const { return {m_entities, index}; }
    entity_ptr engine2D::operator[](std::size_t index) { return {m_entities, index}; }

    const std::vector<entity2D> &engine2D::entities() const { return m_entities; }
    std::size_t engine2D::size() const { return m_entities.size(); }

    const rk::integrator &engine2D::integrator() const { return m_integ; }
    rk::integrator &engine2D::integrator() { return m_integ; }

    const std::vector<float> &engine2D::state() const { return m_state; }

    const compeller2D &engine2D::compeller() const { return m_compeller; }
    compeller2D &engine2D::compeller() { return m_compeller; }

    const collider2D &engine2D::collider() const { return m_collider; }
    collider2D &engine2D::collider() { return m_collider; }

    float engine2D::elapsed() const { return m_t; }

    float engine2D::timestep() const { return m_dt; }
    void engine2D::timestep(float timestep) { m_dt = timestep; }

}