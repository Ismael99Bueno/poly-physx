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
        return m_integ.raw_forward(m_t, m_dt, *this, ode);
    }
    bool engine2D::reiterative_forward(const std::size_t reiterations)
    {
        return m_integ.reiterative_forward(m_t, m_dt, *this, ode, reiterations);
    }
    bool engine2D::embedded_forward()
    {
        return m_integ.embedded_forward(m_t, m_dt, *this, ode);
    }

    void engine2D::reset_accelerations()
    {
        for (entity2D &e : m_entities)
            e.m_accel = {{0.f, 0.f}, 0.f};
    }

    entity_ptr engine2D::add_entity(const vec2 &pos,
                                    const vec2 &vel,
                                    const float angpos,
                                    const float angvel,
                                    const float mass,
                                    const float charge)
    {
        entity2D &e = m_entities.emplace_back(pos, vel, angpos, angvel, mass, charge);
        e.m_buffer = utils::const_vec_ptr(m_state, m_state.size());
        m_state.insert(m_state.end(), {pos.x, pos.y, angpos, vel.x, vel.y, angvel});
        m_collider.add_entity({m_entities, m_entities.size() - 1});

        e.retrieve();
        m_integ.resize();
        return entity_ptr(m_entities, m_entities.size() - 1);
    }

    void engine2D::add_constrain(const constrain_interface &c) { m_compeller.add_constrain(c); }

    const_entity_ptr engine2D::get(const std::size_t index) const { return const_entity_ptr(m_entities, index); }
    entity_ptr engine2D::get(const std::size_t index) { return entity_ptr(m_entities, index); }
    float engine2D::elapsed() const { return m_t; }
    rk::integrator &engine2D::integrator() { return m_integ; }
}