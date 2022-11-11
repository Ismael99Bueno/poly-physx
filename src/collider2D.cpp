#include "collider2D.hpp"
#include <set>
#include <limits>
#include <cmath>

#define POS_PER_ENTITY 3

namespace physics
{
    collider2D::collider2D(const float stiffness,
                           const float dampening,
                           const std::size_t allocations) : m_stiffness(stiffness), m_dampening(dampening)
    {
        m_entities.reserve(allocations);
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

    std::vector<collider2D::collision_pair> collider2D::detect_collisions() const
    {
        std::vector<collision_pair> collisions;
        collisions.reserve(m_entities.size());

        std::vector<const_entity_ptr> eligible;
        eligible.reserve(6);
        for (const interval &itrv : m_intervals)
            if (itrv.type() == interval::LOWER)
                eligible.emplace_back(itrv.entity());
            else
            {
                for (const const_entity_ptr &entity : eligible)
                    if (entity != itrv.entity() &&
                        entity->bounding_box().overlaps(itrv.entity()->bounding_box()) &&
                        entity->shape().overlaps(itrv.entity()->shape()))
                        collisions.emplace_back(entity, itrv.entity());
                eligible.clear();
            }
        return collisions;
    }

    void collider2D::load_collisions(const std::vector<collision_pair> &collisions,
                                     std::vector<float> &stchanges) const
    {
        for (const collision_pair &pair : collisions)
        {
            const const_entity_ptr &e1 = pair.e1, &e2 = pair.e2;

            const auto [touch1, touch2] = touch_points(e1, e2);
            if ((touch1 - touch2).dot(e1->pos() - e2->pos()) > 0.f)
                continue;

            const std::array<float, VAR_PER_ENTITY> accels = state_changes_upon_collision(e1, e2, touch1, touch2);
            for (std::size_t i = 0; i < POS_PER_ENTITY; i++)
            {
                stchanges[e1.index() * VAR_PER_ENTITY + i + POS_PER_ENTITY] += accels[i];
                stchanges[e2.index() * VAR_PER_ENTITY + i + POS_PER_ENTITY] += accels[i + POS_PER_ENTITY];
            }
        }
    }

    std::pair<vec2, vec2> collider2D::touch_points(const const_entity_ptr &e1, const const_entity_ptr &e2) const
    {
        const auto [sep11, sep12] = e1->shape().separation_points(e2->shape());
        const auto [sep21, sep22] = e2->shape().separation_points(e1->shape());

        const float d1 = sep11.sq_dist(sep12),
                    d2 = sep21.sq_dist(sep22);

        return {d1 < d2 ? sep11 : sep22, d1 < d2 ? sep12 : sep21};
    }

    std::array<float, VAR_PER_ENTITY> collider2D::state_changes_upon_collision(const const_entity_ptr &e1,
                                                                               const const_entity_ptr &e2,
                                                                               const vec2 &touch1,
                                                                               const vec2 &touch2) const
    {

        const vec2 rel1 = touch1 - e1->shape().centroid(),
                   rel2 = touch2 - e2->shape().centroid();

        const vec2 vel1 = e1->vel() + rel1.norm() * e1->angvel() * vec2(-std::sin(rel1.angle()), std::cos(rel1.angle())),
                   vel2 = e2->vel() + rel2.norm() * e2->angvel() * vec2(-std::sin(rel2.angle()), std::cos(rel2.angle()));
        const vec2 accel = m_stiffness * (touch2 - touch1) + m_dampening * (vel2 - vel1);
        const float angaccel1 = rel1.cross(accel) / rel1.sq_norm(), angaccel2 = accel.cross(rel2) / rel2.sq_norm();
        return {accel.x, accel.y, angaccel1, -accel.x, -accel.y, angaccel2};
    }
}