#include "collider2D.hpp"
#include <set>
#include <limits>
#include <cmath>

namespace physics
{
    collider2D::collider2D(std::vector<entity2D> &entities,
                           const std::size_t allocations) : m_buffer(entities)
    {
        m_entities.reserve(allocations);
        m_intervals.reserve(allocations);
        m_constrains.reserve(allocations);
    }

    collider2D::interval::interval(const entity_ptr &e, const end end_type) : m_entity(e), m_end(end_type) {}

    const entity_ptr &collider2D::interval::entity() const { return m_entity; }

    float collider2D::interval::value() const
    {
        return (m_end == LOWER) ? m_entity->bounding_box().min().x : m_entity->bounding_box().max().x;
    }

    collider2D::interval::end collider2D::interval::type() const { return m_end; }

    void collider2D::collision_constrain::compute_touch_points()
    {
        const const_entity_ptr &e1 = m_entities[0], &e2 = m_entities[1];
        const auto [sep11, sep12] = e1->shape().separation_points(e2->shape());
        const auto [sep21, sep22] = e2->shape().separation_points(e1->shape());
        const float d1 = sep11.sq_dist(sep12), d2 = sep21.sq_dist(sep22);
        m_touch1 = d1 < d2 ? sep11 : sep21;
        m_touch2 = d1 < d2 ? sep12 : sep22;
    }

    std::pair<vec2, vec2> collider2D::collision_constrain::compute_touch_velocities() const
    {
        const const_entity_ptr &e1 = m_entities[0], &e2 = m_entities[1];
        const vec2 rel_vx1 = (m_touch1 - e1->shape().centroid()),
                   rel_vx2 = (m_touch2 - e2->shape().centroid());
        const float a1 = rel_vx2.angle(), a2 = rel_vx2.angle();
        return {e1->vel() + rel_vx1.norm() * e1->angvel() * vec2(-std::sin(a1), std::cos(a1)),
                e2->vel() + rel_vx2.norm() * e2->angvel() * vec2(-std::sin(a2), std::cos(a2))};
    }

    float collider2D::collision_constrain::constrain(const std::array<const_entity_ptr, 2> &entities) const
    {
        return m_touch1.sq_dist(m_touch2);
    }

    float collider2D::collision_constrain::constrain_derivative(const std::array<const_entity_ptr, 2> &entities) const
    {
        const auto [v1, v2] = compute_touch_velocities();
        return 2.f * (m_touch1 - m_touch2).dot(v1 - v2);
    }

    void collider2D::add(const std::size_t index)
    {
        m_entities.emplace_back(m_buffer, index);
        const entity_ptr &e = m_entities[m_entities.size() - 1];
        m_intervals.emplace_back(e, interval::LOWER);
        m_intervals.emplace_back(e, interval::HIGHER);
    }

    const std::vector<collider2D::collision_constrain> &collider2D::build_collision_constrains()
    {
        sort_entities();
        m_constrains.clear();

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
                        m_constrains.emplace_back(std::array<entity_ptr, 2>({entity, itrv.entity()}));
                eligible.clear();
            }
        return m_constrains;
    }

    void collider2D::sort_entities()
    {
        const auto cmp = [](const interval &itrv1, const interval &itrv2)
        { return itrv1.value() < itrv2.value(); };
        std::sort(m_intervals.begin(), m_intervals.end(), cmp);
    }
}