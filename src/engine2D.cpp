#include "engine2D.hpp"

namespace physics
{
    engine2D::engine2D(const std::size_t allocations)
    {
        m_entities.reserve(allocations);
        m_state.reserve(8 * allocations);
    }

    void engine2D::retrieve()
    {
        for (entity2D &e : m_entities)
            e.retrieve();
    }

    entity_ptr engine2D::add(const vec2 &pos = {0.f, 0.f},
                             const vec2 &vel = {0.f, 0.f},
                             const float angpos = 0.f,
                             const float angvel = 0.f,
                             const float mass = 1.f,
                             const float charge = 1.f)
    {
        m_entities.emplace_back(pos, vel, angpos, angvel, mass, charge);
        m_entities[m_entities.size() - 1].m_buffer = utils::const_vec_ptr(m_state, m_state.size());
        m_state.insert(m_state.end(), {pos.x, pos.y, vel.x, vel.y, angpos, angvel, mass, charge});
        return entity_ptr(m_entities, m_entities.size() - 1);
    }

    const_entity_ptr engine2D::get(const std::size_t index) const { return const_entity_ptr(m_entities, index); }
    entity_ptr engine2D::get(const std::size_t index) { return entity_ptr(m_entities, index); }
}