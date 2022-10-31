#include "collider2D.hpp"
#include <set>

namespace physics
{
    collider2D::collider2D(std::vector<entity2D> &entities,
                           const std::size_t allocations) : m_buffer(entities)
    {
        m_entities.reserve(allocations);
        m_intervals.reserve(allocations);
        m_collisions.reserve(allocations);
    }

    collider2D::collision_pair::collision_pair(const entity_ptr &e1,
                                               const entity_ptr &e2) : e1(e1), e2(e2) {}

    collider2D::interval::interval(const entity_ptr &e, const end end_type) : m_entity(e), m_end(end_type) {}

    const entity_ptr &collider2D::interval::entity() const { return m_entity; }

    float collider2D::interval::value() const
    {
        return (m_end == LOWER) ? m_entity->bounding_box().min().x : m_entity->bounding_box().max().x;
    }

    collider2D::interval::end collider2D::interval::type() const { return m_end; }

    void collider2D::add(const std::size_t index)
    {
        m_entities.emplace_back(m_buffer, index);
        const entity_ptr &e = m_entities[m_entities.size() - 1];
        m_intervals.emplace_back(e, interval::LOWER);
        m_intervals.emplace_back(e, interval::HIGHER);
    }

    void collider2D::detect_collisions()
    {
        sort_entities();
        m_collisions.clear();

        std::vector<entity_ptr> eligible;
        eligible.reserve(6);
        for (const interval &itrv : m_intervals)
            if (itrv.type() == interval::LOWER)
                eligible.emplace_back(itrv.entity());
            else
            {
                for (const entity_ptr &entity : eligible)
                    if (entity != itrv.entity() &&
                        entity->bounding_box().overlaps(itrv.entity()->bounding_box()) &&
                        entity->shape().overlaps(itrv.entity()->shape()))
                        m_collisions.emplace_back(entity, itrv.entity());
                eligible.clear();
            }
    }

    void collider2D::sort_entities()
    {
        const auto cmp = [](const interval &itrv1, const interval &itrv2)
        { return itrv1.value() < itrv2.value(); };
        std::sort(m_intervals.begin(), m_intervals.end(), cmp);
    }
}