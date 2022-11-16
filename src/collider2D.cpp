#include "collider2D.hpp"
#include "debug.h"
#include <set>
#include <limits>
#include <cmath>
#include <algorithm>

#define POS_PER_ENTITY 3
#define EPA_EPSILON 1.e-3f

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

    void collider2D::add_entity(const const_entity_ptr &e)
    {
        m_intervals.emplace_back(e, interval::LOWER);
        m_intervals.emplace_back(e, interval::HIGHER);
    }

    void collider2D::solve_and_load_collisions(std::vector<float> &stchanges)
    {
        sort_intervals();
        const std::vector<collision> collisions = detect_collisions();
        load_collisions(collisions, stchanges);
    }

    float collider2D::stiffness() const { return m_stiffness; }
    float collider2D::dampening() const { return m_dampening; }

    void collider2D::stiffness(float stiffness) { m_stiffness = stiffness; }
    void collider2D::dampening(float dampening) { m_dampening = dampening; }

    void collider2D::sort_intervals()
    {
        const auto cmp = [](const interval &itrv1, const interval &itrv2)
        { return itrv1.value() < itrv2.value(); };
        std::sort(m_intervals.begin(), m_intervals.end(), cmp);
    }

    std::vector<collider2D::collision> collider2D::detect_collisions()
    {
        std::vector<collision> collisions;
        std::vector<const_entity_ptr> eligible;
        sort_intervals();

        eligible.reserve(6);
        collisions.reserve(m_intervals.size() / 2);
        for (const interval &itrv : m_intervals)
            if (itrv.type() == interval::LOWER)
            {
                for (const const_entity_ptr &e : eligible)
                {
                    collision c;
                    if (e != itrv.entity() &&
                        e->bounding_box().overlaps(itrv.entity()->bounding_box()) &&
                        gjk_epa(e, itrv.entity(), c))
                        collisions.emplace_back(c);
                }
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

    void collider2D::load_collisions(const std::vector<collision> &collisions,
                                     std::vector<float> &stchanges) const
    {
        for (const collision &c : collisions)
        {
            const std::array<float, VAR_PER_ENTITY> forces = forces_upon_collision(c);
            for (std::size_t i = 0; i < POS_PER_ENTITY; i++)
            {
                if (c.e1->dynamic())
                    stchanges[c.e1.index() * VAR_PER_ENTITY + i + POS_PER_ENTITY] += forces[i];
                if (c.e2->dynamic())
                    stchanges[c.e2.index() * VAR_PER_ENTITY + i + POS_PER_ENTITY] += forces[i + POS_PER_ENTITY];
            }
        }
    }

    std::array<float, VAR_PER_ENTITY> collider2D::forces_upon_collision(const collision &c) const
    {
        const vec2 rel1 = c.touch1 - c.e1->shape().centroid(),
                   rel2 = c.touch2 - c.e2->shape().centroid();

        const vec2 vel1 = c.e1->vel() + rel1.norm() * c.e1->angvel() * vec2(-std::sin(rel1.angle()), std::cos(rel1.angle())),
                   vel2 = c.e2->vel() + rel2.norm() * c.e2->angvel() * vec2(-std::sin(rel2.angle()), std::cos(rel2.angle()));

        const float director = (c.touch1 - c.touch2).dot(c.e1->pos() - c.e2->pos());
        const float sign = director > 0.f ? -1.f : 1.f;

        const vec2 force = (m_stiffness * (c.touch2 - c.touch1) + m_dampening * (vel2 - vel1)) * sign;
        const float torque1 = rel1.cross(force), torque2 = force.cross(rel2);
        return {force.x, force.y, torque1, -force.x, -force.y, torque2};
    }

    bool collider2D::gjk_epa(const const_entity_ptr &e1, const const_entity_ptr &e2, collision &c)
    {
        std::vector<vec2> simplex;
        if (!gjk(e1->shape(), e2->shape(), simplex))
            return false;
        const vec2 mtv = epa(e1->shape(), e2->shape(), simplex);
        const auto [t1, t2] = touch_points(e1->shape(), e2->shape(), mtv);
        c = {e1, e2, t1, t2};
        return true;
    }

    bool collider2D::gjk(const geo::polygon2D &poly1, const geo::polygon2D &poly2, std::vector<vec2> &simplex)
    {
        vec2 dir = poly2.centroid() - poly1.centroid();
        simplex.reserve(3);
        const vec2 supp = poly1.support_vertex(dir) - poly2.support_vertex(-dir);
        dir = -supp;
        simplex.emplace_back(supp);

        for (;;)
        {
            const vec2 A = poly1.support_vertex(dir) - poly2.support_vertex(-dir);
            if (A.dot(dir) < 0.f)
                return false;
            simplex.emplace_back(A);
            if (simplex.size() == 2)
                line_case(simplex, dir);
            else if (triangle_case(simplex, dir))
                return true;
        }
    }
    void collider2D::line_case(const std::vector<vec2> &simplex, vec2 &dir)
    {
        const vec2 AB = simplex[0] - simplex[1], AO = -simplex[1];
        dir = vec2::triple_cross(AB, AO, AB);
    }
    bool collider2D::triangle_case(std::vector<vec2> &simplex, vec2 &dir)
    {
        const vec2 AB = simplex[1] - simplex[2], AC = simplex[0] - simplex[2], AO = -simplex[2];
        const vec2 ABperp = vec2::triple_cross(AC, AB, AB);
        if (ABperp.dot(AO) > 0.f)
        {
            simplex.erase(simplex.begin());
            return false;
        }
        const vec2 ACperp = vec2::triple_cross(AB, AC, AC);
        if (ACperp.dot(AO) > 0.f)
        {
            simplex.erase(simplex.begin() + 1);
            dir = ACperp;
            return false;
        }
        return true;
    }

    vec2 collider2D::epa(const geo::polygon2D &poly1, const geo::polygon2D &poly2, std::vector<vec2> &simplex)
    {
        float min_dist = std::numeric_limits<float>::max();
        vec2 mtv;
        for (;;)
        {
            std::size_t min_index;
            for (std::size_t i = 0; i < simplex.size(); i++)
            {
                const std::size_t j = (i + 1) % simplex.size();

                const vec2 &p1 = simplex[i], &p2 = simplex[j];
                const vec2 edge = p2 - p1;

                const vec2 normal = vec2(edge.y, -edge.x).normalized();
                const float dist = normal.dot(p1);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_index = j;
                    mtv = normal;
                }
            }
            const vec2 support = poly1.support_vertex(mtv) - poly2.support_vertex(-mtv);
            const float sup_dist = mtv.dot(support);
            const float diff = std::abs(sup_dist - min_dist);
            if (diff <= EPA_EPSILON)
                break;
            simplex.insert(simplex.begin() + min_index, support);
            min_dist = std::numeric_limits<float>::max();
        }
        return mtv * min_dist;
    }

    std::pair<vec2, vec2> collider2D::touch_points(const geo::polygon2D &poly1,
                                                   const geo::polygon2D &poly2,
                                                   const vec2 &mtv)
    {
        vec2 t1, t2;
        float min_dist = std::numeric_limits<float>::max();
        for (const vec2 &v : poly1.vertices())
        {
            const vec2 towards = poly2.towards_closest_edge_from(v - mtv);
            const float dist = towards.sq_norm();
            if (min_dist > dist)
            {
                min_dist = dist;
                t1 = v;
                t2 = v - mtv;
            }
        }
        for (const vec2 &v : poly2.vertices())
        {
            const vec2 towards = poly1.towards_closest_edge_from(v + mtv);
            const float dist = towards.sq_norm();
            if (min_dist > dist)
            {
                min_dist = dist;
                t2 = v;
                t1 = v + mtv;
            }
        }
        return {t1, t2};
    }
}