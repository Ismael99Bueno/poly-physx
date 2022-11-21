#include "engine2D.hpp"
#include "ode2D.hpp"

namespace physics
{
    engine2D::engine2D(const rk::tableau &table,
                       const float dt,
                       const std::size_t allocations) : m_dt(dt),
                                                        m_integ(table, m_state),
                                                        m_compeller(m_entities)
    {
        m_entities.reserve(allocations);
        m_state.reserve(VAR_PER_ENTITY * allocations);
        m_integ.reserve(VAR_PER_ENTITY * allocations);
    }

    void engine2D::retrieve(const std::vector<float> &state)
    {
        for (std::size_t i = 0; i < m_entities.size(); i++)
            m_entities[i].retrieve(utils::const_vec_ptr(state, VAR_PER_ENTITY * i));
    }

    void engine2D::retrieve() { retrieve(m_state); }

    bool engine2D::raw_forward()
    {
        const bool valid = m_integ.raw_forward(m_t, m_dt, *this, ode);
        reset_forces();
        return valid;
    }
    bool engine2D::reiterative_forward(const std::size_t reiterations)
    {
        const bool valid = m_integ.reiterative_forward(m_t, m_dt, *this, ode, reiterations);
        reset_forces();
        return valid;
    }
    bool engine2D::embedded_forward()
    {
        const bool valid = m_integ.embedded_forward(m_t, m_dt, *this, ode);
        reset_forces();
        return valid;
    }

    void engine2D::reset_forces()
    {
        for (entity2D &e : m_entities)
        {
            e.m_force = {0.f, 0.f};
            e.m_torque = 0.f;
        }
    }

    entity_ptr engine2D::add_entity(const body2D &body,
                                    const std::vector<vec2> &vertices)
    {
        entity2D &e = m_entities.emplace_back(body, vertices);
        e.m_buffer = utils::vec_ptr(m_state, m_state.size());
        m_state.insert(m_state.end(), {body.pos().x, body.pos().y, body.angpos(),
                                       body.vel().x, body.vel().y, body.angvel()});
        m_collider.add_entity({m_entities, m_entities.size() - 1});

        e.retrieve();
        m_integ.resize();
        return entity_ptr(m_entities, m_entities.size() - 1);
    }
    entity_ptr engine2D::add_entity(const vec2 &pos,
                                    const vec2 &vel,
                                    const float angpos,
                                    const float angvel,
                                    const float mass,
                                    const float charge,
                                    const std::vector<vec2> &vertices)
    {
        return add_entity(body2D(pos, vel, angpos, angvel, mass, charge), vertices);
    }

    void engine2D::add_constrain(const constrain_interface &c) { m_compeller.add_constrain(c); }

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