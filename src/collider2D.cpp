#include "collider2D.hpp"
#include <set>
#include <limits>
#include <cmath>
#include <algorithm>

#define POS_PER_ENTITY 3

namespace physics
{
    collider2D::collider2D(const float stiffness,
                           const float dampening,
                           const std::size_t allocations) : m_stiffness(stiffness), m_dampening(dampening)
    {
        m_intervals.reserve(allocations);
    }

    collider2D::interval::interval(const const_entity_ptr &e, const end end_type) : m_entity(e), m_end(end_type) {}

    const const_entity_ptr &collider2D::interval::entity() const { return m_entity; }

    float collider2D::interval::value() const
    {
        return (m_end == LOWER) ? m_entity->bounding_box().min().x : m_entity->bounding_box().max().x;
    }

    collider2D::interval::end collider2D::interval::type() const { return m_end; }

    collider2D::collision_pair::collision_pair(const const_entity_ptr &e1,
                                               const const_entity_ptr &e2) : e1(e1), e2(e2) {}

    void collider2D::add_entity(const const_entity_ptr &e)
    {
        m_intervals.emplace_back(e, interval::LOWER);
        m_intervals.emplace_back(e, interval::HIGHER);
    }

    void collider2D::solve_and_load_collisions(std::vector<float> &stchanges)
    {
        sort_intervals();
        const std::vector<collision_pair> collisions = detect_collisions();
        load_collisions(collisions, stchanges);
    }

    void collider2D::sort_intervals()
    {
        const auto cmp = [](const interval &itrv1, const interval &itrv2)
        { return itrv1.value() < itrv2.value(); };
        std::sort(m_intervals.begin(), m_intervals.end(), cmp);
    }

    std::vector<collider2D::collision_pair> collider2D::detect_collisions()
    {
        std::vector<collision_pair> collisions;
        std::vector<const_entity_ptr> eligible;
        sort_intervals();

        eligible.reserve(6);
        collisions.reserve(m_intervals.size() / 2);
        for (const interval &itrv : m_intervals)
            if (itrv.type() == interval::LOWER)
            {
                for (const const_entity_ptr &e : eligible)
                    if (e != itrv.entity() &&
                        e->bounding_box().overlaps(itrv.entity()->bounding_box()) &&
                        e->shape().overlaps(itrv.entity()->shape()))
                        collisions.emplace_back(e, itrv.entity());
                eligible.emplace_back(itrv.entity());
            }
            else
                for (auto it = eligible.begin(); it != eligible.end(); ++it)
                    if (*it == itrv.entity())
                    {
                        eligible.erase(it);
                        break;
                    }
        return collisions;
    }

    void collider2D::load_collisions(const std::vector<collision_pair> &collisions,
                                     std::vector<float> &stchanges) const
    {
        for (const collision_pair &pair : collisions)
        {
            const const_entity_ptr &e1 = pair.e1, &e2 = pair.e2;
            const std::array<float, VAR_PER_ENTITY> forces = forces_upon_collision(e1, e2);
            for (std::size_t i = 0; i < POS_PER_ENTITY; i++)
            {
                if (e1->dynamic())
                    stchanges[e1.index() * VAR_PER_ENTITY + i + POS_PER_ENTITY] += forces[i];
                if (e2->dynamic())
                    stchanges[e2.index() * VAR_PER_ENTITY + i + POS_PER_ENTITY] += forces[i + POS_PER_ENTITY];
            }
        }
    }

    std::array<float, VAR_PER_ENTITY> collider2D::forces_upon_collision(const const_entity_ptr &e1,
                                                                        const const_entity_ptr &e2) const
    {
        const auto [touch1, touch2] = geo::polygon2D::touch_points(e1->shape(), e2->shape());

        const vec2 rel1 = touch1 - e1->shape().centroid(),
                   rel2 = touch2 - e2->shape().centroid();

        const vec2 vel1 = e1->vel() + rel1.norm() * e1->angvel() * vec2(-std::sin(rel1.angle()), std::cos(rel1.angle())),
                   vel2 = e2->vel() + rel2.norm() * e2->angvel() * vec2(-std::sin(rel2.angle()), std::cos(rel2.angle()));

        const float director = (touch1 - touch2).dot(e1->pos() - e2->pos());
        const float sign = director > 0.f ? -1.f : 1.f;

        const vec2 force = (m_stiffness * (touch2 - touch1) + m_dampening * (vel2 - vel1)) * sign;
        const float torque1 = rel1.cross(force), torque2 = force.cross(rel2);
        return {force.x, force.y, torque1, -force.x, -force.y, torque2};
    }
}