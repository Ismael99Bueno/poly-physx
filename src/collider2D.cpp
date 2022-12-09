#include "collider2D.hpp"
#include "debug.h"
#include "perf.hpp"
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_set>

#define EPA_EPSILON 1.e-3f

namespace phys
{
    collider2D::collider2D(const std::vector<entity2D> &entities,
                           const std::size_t allocations) : m_entities(entities),
                                                            m_quad_tree({0.f, 0.f}, {192.f, 128.f})
    {
        m_intervals.reserve(allocations);
    }

    collider2D::interval::interval(const const_entity_ptr &e, const end end_type) : m_entity(e), m_end(end_type) {}

    const entity2D *collider2D::interval::entity() const { return m_entity.raw(); }

    float collider2D::interval::value() const
    {
        return (m_end == LOWER) ? m_entity->bounding_box().min().x : m_entity->bounding_box().max().x;
    }

    collider2D::interval::end collider2D::interval::type() const { return m_end; }
    bool collider2D::interval::try_validate() { return m_entity.try_validate(); }

    void collider2D::add_entity_intervals(const const_entity_ptr &e)
    {
        m_intervals.emplace_back(e, interval::LOWER);
        m_intervals.emplace_back(e, interval::HIGHER);
    }

    void collider2D::solve_and_load_collisions(std::vector<float> &stchanges)
    {
        const std::vector<collision> collisions = detect_collisions();
        load_collisions(collisions, stchanges);
    }

    void collider2D::update_quad_tree()
    {
        m_quad_tree.update(m_entities);
        m_qt_build_calls = 0;
    }

    void collider2D::rebuild_quad_tree()
    {
        m_quad_tree.rebuild(m_entities);
        m_qt_build_calls = 0;
    }

    bool collider2D::validate()
    {
        std::vector<std::size_t> invalids;
        invalids.reserve(2);
        for (std::size_t i = 0; i < m_intervals.size(); i++)
            if (!m_intervals[i].try_validate())
                invalids.emplace_back(i);
        for (const std::size_t index : invalids)
        {
            m_intervals[index] = m_intervals.back();
            m_intervals.pop_back();
        }
        DBG_LOG_IF(invalids.empty() && !m_intervals.empty(), "Validate method did not find any invalid entity pointers.\n")
        DBG_LOG_IF(!invalids.empty() && !m_intervals.empty(), "Validate method found %zu invalid entity pointers.\n", invalids.size())
        return !invalids.empty();
    }

    std::vector<collider2D::collision> collider2D::detect_collisions()
    {
        std::vector<collider2D::collision> collisions;
        collisions.reserve(m_entities.size() / 2);
        switch (m_coldet_method)
        {
        case BRUTE_FORCE:
            brute_force_coldet(collisions);
            break;
        case SORT_AND_SWEEP:
            sort_and_sweep_coldet(collisions);
            break;
        case QUAD_TREE:
            quad_tree_coldet(collisions);
            break;
        }
        return collisions;
    }

    float collider2D::stiffness() const { return m_stiffness; }
    float collider2D::dampening() const { return m_dampening; }

    void collider2D::stiffness(float stiffness) { m_stiffness = stiffness; }
    void collider2D::dampening(float dampening) { m_dampening = dampening; }

    collider2D::coldet_method collider2D::coldet() const { return m_coldet_method; }
    void collider2D::coldet(coldet_method coldet) { m_coldet_method = coldet; }

    const quad_tree2D &collider2D::quad_tree() const { return m_quad_tree; }
    quad_tree2D &collider2D::quad_tree() { return m_quad_tree; }

    std::size_t collider2D::quad_tree_build_period() const { return m_qt_build_period; }
    void collider2D::quad_tree_build_period(const std::size_t period) { m_qt_build_period = period; }

    void collider2D::sort_intervals()
    {
        PERF_FUNCTION()
        const auto cmp = [](const interval &itrv1, const interval &itrv2)
        { return itrv1.value() < itrv2.value(); };
        std::sort(m_intervals.begin(), m_intervals.end(), cmp);
    }

    bool collider2D::collide(const entity2D *e1, const entity2D *e2, collision *c)
    {
        return e1 != e2 && e1->bounding_box().overlaps(e2->bounding_box()) && gjk_epa(e1, e2, c);
    }

    void collider2D::brute_force_coldet(std::vector<collision> &collisions) const
    {
        PERF_FUNCTION()
        for (std::size_t i = 0; i < m_entities.size(); i++)
            for (std::size_t j = i + 1; j < m_entities.size(); j++)
            {
                collision c;
                if (collide(&m_entities[i], &m_entities[j], &c))
                    collisions.emplace_back(c);
            }
    }

    void collider2D::sort_and_sweep_coldet(std::vector<collision> &collisions)
    {
        PERF_FUNCTION()
        std::unordered_set<const entity2D *> eligible;
        sort_intervals();

        eligible.reserve(6);
        for (const interval &itrv : m_intervals)
            if (itrv.type() == interval::LOWER)
            {
                for (const entity2D *e : eligible)
                {
                    collision c;
                    if (collide(e, itrv.entity(), &c))
                        collisions.emplace_back(c);
                }
                eligible.insert(itrv.entity());
            }
            else
                eligible.erase(itrv.entity());
    }

    void collider2D::quad_tree_coldet(std::vector<collision> &collisions)
    {
        PERF_FUNCTION()

        if (m_qt_build_calls++ >= m_qt_build_period)
            update_quad_tree();

        std::vector<const std::vector<const_entity_ptr> *> partitions;
        partitions.reserve(20);
        m_quad_tree.partitions(partitions);
        for (const std::vector<const_entity_ptr> *partition : partitions)
            for (std::size_t i = 0; i < partition->size(); i++)
                for (std::size_t j = i + 1; j < partition->size(); j++)
                {
                    collision c;
                    if (collide(partition->operator[](i).raw(), partition->operator[](j).raw(), &c))
                        collisions.emplace_back(c);
                }
    }

    void collider2D::load_collisions(const std::vector<collision> &collisions,
                                     std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        for (const collision &c : collisions)
        {
            const std::array<float, 6> forces = forces_upon_collision(c);
            for (std::size_t i = 0; i < 3; i++)
            {
                if (c.e1->dynamic())
                    stchanges[c.e1->index() * 6 + i + 3] += forces[i];
                if (c.e2->dynamic())
                    stchanges[c.e2->index() * 6 + i + 3] += forces[i + 3];
            }
        }
    }

    std::array<float, 6> collider2D::forces_upon_collision(const collision &c) const
    {
        const alg::vec2 rel1 = c.touch1 - c.e1->pos(),
                        rel2 = c.touch2 - c.e2->pos();

        const alg::vec2 vel1 = c.e1->vel_at(rel1),
                        vel2 = c.e2->vel_at(rel2);

        const float director = (c.touch1 - c.touch2).dot(c.e1->pos() - c.e2->pos());
        const float mass_factor = (c.e1->mass() + c.e2->mass()) / 2.f;

        const alg::vec2 force = (m_stiffness * (c.touch2 - c.touch1) + m_dampening * (vel2 - vel1)) * mass_factor;
        const float torque1 = rel1.cross(force), torque2 = force.cross(rel2);
        return {force.x, force.y, torque1, -force.x, -force.y, torque2};
    }

    bool collider2D::gjk_epa(const entity2D *e1, const entity2D *e2, collision *c)
    {
        std::vector<alg::vec2> simplex;
        if (!gjk(e1->shape(), e2->shape(), simplex))
            return false;
        const alg::vec2 mtv = epa(e1->shape(), e2->shape(), simplex);
        const auto [t1, t2] = touch_points(e1->shape(), e2->shape(), mtv);
        *c = {e1, e2, t1, t2};
        return true;
    }

    bool collider2D::gjk(const geo::polygon2D &poly1, const geo::polygon2D &poly2, std::vector<alg::vec2> &simplex)
    {
        PERF_FUNCTION()
        alg::vec2 dir = poly2.centroid() - poly1.centroid();
        simplex.reserve(3);
        const alg::vec2 supp = poly1.support_vertex(dir) - poly2.support_vertex(-dir);
        dir = -supp;
        simplex.emplace_back(supp);

        for (;;)
        {
            const alg::vec2 A = poly1.support_vertex(dir) - poly2.support_vertex(-dir);
            if (A.dot(dir) < 0.f)
                return false;
            simplex.emplace_back(A);
            if (simplex.size() == 2)
                line_case(simplex, dir);
            else if (triangle_case(simplex, dir))
                return true;
        }
    }
    void collider2D::line_case(const std::vector<alg::vec2> &simplex, alg::vec2 &dir)
    {
        const alg::vec2 AB = simplex[0] - simplex[1], AO = -simplex[1];
        dir = alg::vec2::triple_cross(AB, AO, AB);
    }
    bool collider2D::triangle_case(std::vector<alg::vec2> &simplex, alg::vec2 &dir)
    {
        const alg::vec2 AB = simplex[1] - simplex[2], AC = simplex[0] - simplex[2], AO = -simplex[2];
        const alg::vec2 ABperp = alg::vec2::triple_cross(AC, AB, AB);
        if (ABperp.dot(AO) > 0.f)
        {
            simplex.erase(simplex.begin());
            return false;
        }
        const alg::vec2 ACperp = alg::vec2::triple_cross(AB, AC, AC);
        if (ACperp.dot(AO) > 0.f)
        {
            simplex.erase(simplex.begin() + 1);
            dir = ACperp;
            return false;
        }
        return true;
    }

    alg::vec2 collider2D::epa(const geo::polygon2D &poly1, const geo::polygon2D &poly2, std::vector<alg::vec2> &simplex)
    {
        PERF_FUNCTION()
        float min_dist = std::numeric_limits<float>::max();
        alg::vec2 mtv;
        for (;;)
        {
            std::size_t min_index;
            for (std::size_t i = 0; i < simplex.size(); i++)
            {
                const std::size_t j = (i + 1) % simplex.size();

                const alg::vec2 &p1 = simplex[i], &p2 = simplex[j];
                const alg::vec2 edge = p2 - p1;

                const alg::vec2 normal = alg::vec2(edge.y, -edge.x).normalized();
                const float dist = normal.dot(p1);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_index = j;
                    mtv = normal;
                }
            }
            const alg::vec2 support = poly1.support_vertex(mtv) - poly2.support_vertex(-mtv);
            const float sup_dist = mtv.dot(support);
            const float diff = std::abs(sup_dist - min_dist);
            if (diff <= EPA_EPSILON)
                break;
            simplex.insert(simplex.begin() + min_index, support);
            min_dist = std::numeric_limits<float>::max();
        }
        return mtv * min_dist;
    }

    std::pair<alg::vec2, alg::vec2> collider2D::touch_points(const geo::polygon2D &poly1,
                                                             const geo::polygon2D &poly2,
                                                             const alg::vec2 &mtv)
    {
        PERF_FUNCTION()
        alg::vec2 t1, t2;
        float min_dist = std::numeric_limits<float>::max();
        for (const alg::vec2 &v : poly1.vertices())
        {
            const alg::vec2 towards = poly2.towards_closest_edge_from(v - mtv);
            const float dist = towards.sq_norm();
            if (min_dist > dist)
            {
                min_dist = dist;
                t1 = v;
                t2 = v - mtv;
            }
        }
        for (const alg::vec2 &v : poly2.vertices())
        {
            const alg::vec2 towards = poly1.towards_closest_edge_from(v + mtv);
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