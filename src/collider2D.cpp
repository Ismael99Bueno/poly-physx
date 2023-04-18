#include "collider2D.hpp"
#include "debug.hpp"
#include "perf.hpp"
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <glm/geometric.hpp>
#include <glm/gtx/norm.hpp>
#ifdef WINDOWS
#include <execution>
#endif

#define EPA_EPSILON 1.e-3f

namespace ppx
{
    static float cross(const glm::vec2 &v1, const glm::vec2 &v2) { return v1.x * v2.y - v1.y * v2.x; }
    static glm::vec2 triple_cross(const glm::vec2 &v1, const glm::vec2 &v2, const glm::vec2 &v3)
    {
        const float crs = cross(v1, v2);
        return glm::vec2(-v3.y * crs, v3.x * crs);
    }

    collider2D::collider2D(engine_key,
                           std::vector<entity2D> *entities,
                           const std::size_t allocations,
                           const glm::vec2 &min,
                           const glm::vec2 &max) : m_entities(entities),
                                                   m_quad_tree(collider_key(), min, max)
    {
        m_intervals.reserve(allocations);
    }

    collider2D::interval::interval(const const_entity2D_ptr &e, const end end_type) : m_entity(e), m_end(end_type) {}

    const entity2D *collider2D::interval::entity() const { return m_entity.raw(); }

    float collider2D::interval::value() const
    {
        return (m_end == LOWER) ? m_entity->aabb().min().x : m_entity->aabb().max().x;
    }

    collider2D::interval::end collider2D::interval::type() const { return m_end; }
    bool collider2D::interval::try_validate() { return m_entity.try_validate(); }

    void collider2D::add_entity_intervals(const const_entity2D_ptr &e)
    {
        m_intervals.emplace_back(e, interval::LOWER);
        m_intervals.emplace_back(e, interval::HIGHER);
    }

    void collider2D::solve_and_load_collisions(std::vector<float> &stchanges)
    {
        PERF_FUNCTION()
        if (!m_enabled)
            return;
        switch (m_coldet_method)
        {
        case BRUTE_FORCE:
            brute_force_coldet(stchanges);
            break;
        case SORT_AND_SWEEP:
            sort_and_sweep_coldet(stchanges);
            break;
        case QUAD_TREE:
            quad_tree_coldet(stchanges);
            break;
        }
    }

    void collider2D::update_quad_tree()
    {
        m_quad_tree.update(*m_entities);
    }

    void collider2D::rebuild_quad_tree()
    {
        m_quad_tree.rebuild(*m_entities);
    }

    void collider2D::validate()
    {
        for (auto it = m_intervals.begin(); it != m_intervals.end();)
            if (!it->try_validate())
                it = m_intervals.erase(it);
            else
                ++it;
    }

    void collider2D::write(ini::output &out) const
    {
        out.write("stiffness", m_stiffness);
        out.write("dampening", m_dampening);
        out.write("qt_build_period", m_qt_build_period);
        out.write("coldet_method", m_coldet_method);
        out.write("enabled", m_enabled);
    }

    void collider2D::read(ini::input &in)
    {
        m_stiffness = in.readf32("stiffness");
        m_dampening = in.readf32("dampening");
        m_qt_build_period = in.readui32("qt_build_period");
        m_coldet_method = (coldet_method)in.readi32("coldet_method");
        m_enabled = (bool)in.readi16("enabled");
        rebuild_quad_tree();
    }

    float collider2D::stiffness() const { return m_stiffness; }
    float collider2D::dampening() const { return m_dampening; }

    void collider2D::stiffness(float stiffness) { m_stiffness = stiffness; }
    void collider2D::dampening(float dampening) { m_dampening = dampening; }

    bool collider2D::enabled() const { return m_enabled; }
    void collider2D::enabled(const bool enabled) { m_enabled = enabled; }

    collider2D::coldet_method collider2D::coldet() const { return m_coldet_method; }
    void collider2D::coldet(coldet_method coldet) { m_coldet_method = coldet; }

    const quad_tree2D &collider2D::quad_tree() const { return m_quad_tree; }
    quad_tree2D &collider2D::quad_tree() { return m_quad_tree; }

    std::uint32_t collider2D::quad_tree_build_period() const { return m_qt_build_period; }
    void collider2D::quad_tree_build_period(const std::uint32_t period) { m_qt_build_period = period; }

    void collider2D::sort_intervals()
    {
        const auto cmp = [](const interval &itrv1, const interval &itrv2)
        { return itrv1.value() < itrv2.value(); };
        std::sort(m_intervals.begin(), m_intervals.end(), cmp);
    }

    bool collider2D::collide(const entity2D &e1, const entity2D &e2, collision2D *c) const
    {
        const bool collide = e1 != e2 &&
                             (e1.kinematic() || e2.kinematic()) &&
                             e1.aabb().overlaps(e2.aabb()) &&
                             gjk_epa(e1, e2, c);
        if (collide)
        {
            e1.callbacks().try_enter_or_stay(*c);
            e2.callbacks().try_enter_or_stay({c->incoming, c->other, c->touch2, c->touch1, -c->normal});
            return true;
        }
        e1.callbacks().try_exit({m_entities, e2.index()});
        e2.callbacks().try_exit({m_entities, e1.index()});
        return false;
    }

    void collider2D::brute_force_coldet(std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
#if defined(WINDOWS) && !defined(PERF)
        const auto exec = [this, &stchanges](const ppx::entity2D &e1)
        {
            for (std::size_t j = 0; j < m_entities->size(); j++)
            {
                collision2D c;
                const ppx::entity2D &e2 = (*m_entities)[j];
                if (collide(e1, e2, &c))
                    solve(c, stchanges);
            }
        };
        std::for_each(std::execution::par, m_entities->begin(), m_entities->end(), exec);
#else
        for (std::size_t i = 0; i < m_entities->size(); i++)
            for (std::size_t j = i + 1; j < m_entities->size(); j++)
            {
                collision2D c;
                const ppx::entity2D &e1 = (*m_entities)[i], &e2 = (*m_entities)[j];
                if (collide(e1, e2, &c))
                    solve(c, stchanges);
            }
#endif
    }

    void collider2D::sort_and_sweep_coldet(std::vector<float> &stchanges)
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
                    collision2D c;
                    if (collide(*e, *itrv.entity(), &c))
                        solve(c, stchanges);
                }
                eligible.insert(itrv.entity());
            }
            else
                eligible.erase(itrv.entity());
    }

    void collider2D::quad_tree_coldet(std::vector<float> &stchanges)
    {
        PERF_FUNCTION()
        static std::uint32_t qt_build_calls = 0;
        if (qt_build_calls++ >= m_qt_build_period)
        {
            update_quad_tree();
            qt_build_calls = 0;
        }

        std::vector<const std::vector<const_entity2D_ptr> *> partitions;
        partitions.reserve(20);
        m_quad_tree.partitions(partitions);

#if defined(WINDOWS) && !defined(PERF)
        const auto exec = [this, &stchanges](const std::vector<const_entity2D_ptr> *partition)
        {
            for (std::size_t i = 0; i < partition->size(); i++)
                for (std::size_t j = i + 1; j < partition->size(); j++)
                {
                    collision2D c;
                    const auto &e1 = (*partition)[i], &e2 = (*partition)[j];
                    if (collide(*e1, *e2, &c))
                        solve(c, stchanges);
                }
        };
        std::for_each(std::execution::par, partitions.begin(), partitions.end(), exec);
#else
        for (const std::vector<const_entity2D_ptr> *partition : partitions)
            for (std::size_t i = 0; i < partition->size(); i++)
                for (std::size_t j = i + 1; j < partition->size(); j++)
                {
                    collision2D c;
                    const auto &e1 = (*partition)[i], &e2 = (*partition)[j];
                    if (collide(*e1, *e2, &c))
                        solve(c, stchanges);
                }
#endif
    }

    void collider2D::solve(const collision2D &c,
                           std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        const std::array<float, 6> forces = forces_upon_collision(c);
        for (std::size_t i = 0; i < 3; i++)
        {
            if (c.other->kinematic())
                stchanges[c.other->index() * 6 + i + 3] += forces[i];
            if (c.incoming->kinematic())
                stchanges[c.incoming->index() * 6 + i + 3] += forces[i + 3];
        }
    }

    std::array<float, 6> collider2D::forces_upon_collision(const collision2D &c) const
    {
        PERF_FUNCTION()
        const glm::vec2 rel1 = c.touch1 - c.other->pos(),
                        rel2 = c.touch2 - c.incoming->pos();

        const glm::vec2 vel1 = c.other->vel_at(rel1),
                        vel2 = c.incoming->vel_at(rel2);

        const glm::vec2 force = (m_stiffness * (c.touch2 - c.touch1) + m_dampening * (vel2 - vel1));
        const float torque1 = cross(rel1, force), torque2 = cross(force, rel2);
        return {force.x, force.y, torque1, -force.x, -force.y, torque2};
    }

    bool collider2D::gjk_epa(const entity2D &e1, const entity2D &e2, collision2D *c) const
    {
        PERF_FUNCTION()
        std::vector<glm::vec2> simplex;
        if (!gjk(e1.shape(), e2.shape(), simplex))
            return false;
        const glm::vec2 mtv = epa(e1.shape(), e2.shape(), simplex);
        const auto [t1, t2] = touch_points(e1.shape(), e2.shape(), mtv);
        *c = {{m_entities, e1.index()}, {m_entities, e2.index()}, t1, t2, mtv};
        return true;
    }

    bool collider2D::gjk(const geo::polygon &poly1, const geo::polygon &poly2, std::vector<glm::vec2> &simplex)
    {
        PERF_FUNCTION()
        glm::vec2 dir = poly2.centroid() - poly1.centroid();
        simplex.reserve(3);
        const glm::vec2 supp = poly1.support_vertex(dir) - poly2.support_vertex(-dir);
        dir = -supp;
        simplex.emplace_back(supp);

        for (;;)
        {
            const glm::vec2 A = poly1.support_vertex(dir) - poly2.support_vertex(-dir);
            if (glm::dot(A, dir) <= 0.f)
                return false;
            simplex.emplace_back(A);
            if (simplex.size() == 2)
                line_case(simplex, dir);
            else if (triangle_case(simplex, dir))
                return true;
        }
    }
    void collider2D::line_case(const std::vector<glm::vec2> &simplex, glm::vec2 &dir)
    {
        const glm::vec2 AB = simplex[0] - simplex[1], AO = -simplex[1];
        dir = triple_cross(AB, AO, AB);
    }
    bool collider2D::triangle_case(std::vector<glm::vec2> &simplex, glm::vec2 &dir)
    {
        const glm::vec2 AB = simplex[1] - simplex[2], AC = simplex[0] - simplex[2], AO = -simplex[2];
        const glm::vec2 ABperp = triple_cross(AC, AB, AB);
        if (glm::dot(ABperp, AO) >= 0.f)
        {
            simplex.erase(simplex.begin());
            dir = ABperp;
            return false;
        }
        const glm::vec2 ACperp = triple_cross(AB, AC, AC);
        if (glm::dot(ACperp, AO) >= 0.f)
        {
            simplex.erase(simplex.begin() + 1);
            dir = ACperp;
            return false;
        }
        return true;
    }

    glm::vec2 collider2D::epa(const geo::polygon &poly1, const geo::polygon &poly2, std::vector<glm::vec2> &simplex)
    {
        PERF_FUNCTION()
        DBG_LOG_IF(!geo::polygon(simplex).contains_origin(), "Simplex passed to EPA algorithm does not contain the origin!\nx1: %f, y1: %f\nx2: %f, y2: %f\nx3: %f, y3: %f\n", simplex[0].x, simplex[0].y, simplex[1].x, simplex[1].y, simplex[2].x, simplex[2].y)
        float min_dist = std::numeric_limits<float>::max();
        glm::vec2 mtv(0.f);
        for (;;)
        {
            std::size_t min_index;
            for (std::size_t i = 0; i < simplex.size(); i++)
            {
                const std::size_t j = (i + 1) % simplex.size();

                const glm::vec2 &p1 = simplex[i], &p2 = simplex[j];
                const glm::vec2 edge = p2 - p1;

                glm::vec2 normal = glm::normalize(glm::vec2(edge.y, -edge.x));
                float dist = glm::dot(normal, p1);
                if (dist < 0.f)
                {
                    dist *= -1.f;
                    normal *= -1.f;
                }
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_index = j;
                    mtv = normal;
                }
            }
            const glm::vec2 support = poly1.support_vertex(mtv) - poly2.support_vertex(-mtv);
            const float sup_dist = glm::dot(mtv, support);
            const float diff = std::abs(sup_dist - min_dist);
            if (diff <= EPA_EPSILON)
                break;
            simplex.insert(simplex.begin() + (long)min_index, support);
            min_dist = std::numeric_limits<float>::max();
        }
        return mtv * min_dist;
    }

    std::pair<glm::vec2, glm::vec2> collider2D::touch_points(const geo::polygon &poly1,
                                                             const geo::polygon &poly2,
                                                             const glm::vec2 &mtv)
    {
        PERF_FUNCTION()
        const glm::vec2 sup1 = poly1.support_vertex(mtv),
                        sup2 = poly2.support_vertex(-mtv);
        const float d1 = glm::length2(poly2.towards_closest_edge_from(sup1 - mtv)),
                    d2 = glm::length2(poly1.towards_closest_edge_from(sup2 + mtv));
        if (d1 < d2)
            return std::make_pair(sup1, sup1 - mtv);
        return std::make_pair(sup2 + mtv, sup2);
    }
}