#include "ppx/pch.hpp"
#include "ppx/collider2D.hpp"

#include "geo/intersection.hpp"

#ifdef PPX_WINDOWS
#include <execution>
#endif

namespace ppx
{
    static float cross(const glm::vec2 &v1, const glm::vec2 &v2) { return v1.x * v2.y - v1.y * v2.x; }

    collider2D::collider2D(std::vector<entity2D> *entities,
                           const std::size_t allocations,
                           const glm::vec2 &min,
                           const glm::vec2 &max) : m_entities(entities),
                                                   m_quad_tree(min, max)
    {
        m_intervals.reserve(allocations);
    }

    collider2D::interval::interval(const const_entity2D_ptr &e, const end end_type) : m_entity(e), m_end(end_type) {}

    const entity2D *collider2D::interval::entity() const { return m_entity.raw(); }
    float collider2D::interval::value() const
    {
        const geo::aabb2D &bbox = m_entity->shape().bounding_box();
        return (m_end == LOWER) ? bbox.min().x : bbox.max().x;
    }

    collider2D::interval::end collider2D::interval::type() const { return m_end; }
    bool collider2D::interval::validate() { return m_entity.validate(); }

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
            if (!it->validate())
                it = m_intervals.erase(it);
            else
                ++it;
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
        if (e1 == e2 || (!e1.kinematic() && !e2.kinematic()))
            return false;

        const auto *c1 = e1.shape_if<geo::circle>(),
                   *c2 = e2.shape_if<geo::circle>();
        if (c1 && c2)
        {
            if (!geo::intersect(*c1, *c2))
                return false;
            const glm::vec2 mtv = geo::mtv(*c1, *c2);
            const auto &[contact1, contact2] = geo::contact_points(*c1, *c2);
            *c = {{m_entities, e1.index()}, {m_entities, e2.index()}, contact1, contact2, mtv};
            return true;
        }

        const geo::shape2D &sh1 = e1.shape(),
                           &sh2 = e2.shape();
        if (!geo::may_intersect(sh1, sh2))
            return false;

        std::vector<glm::vec2> simplex;
        simplex.reserve(10);
        if (!geo::gjk(sh1, sh2, simplex))
            return false;

        glm::vec2 mtv;
        if (!geo::epa(sh1, sh2, simplex, mtv))
            return false;
        const auto &[contact1, contact2] = geo::contact_points(sh1, sh2, mtv);
        *c = {{m_entities, e1.index()}, {m_entities, e2.index()}, contact1, contact2, mtv};

        return true;
    }

    void collider2D::try_enter_or_stay_callback(const entity2D &e1, const entity2D &e2, const collision2D &c) const
    {
        e1.events().try_enter_or_stay(c);
        e2.events().try_enter_or_stay({c.incoming, c.current, c.touch2, c.touch1, -c.normal});
    }
    void collider2D::try_exit_callback(const entity2D &e1, const entity2D &e2) const
    {
        e1.events().try_exit({m_entities, e2.index()});
        e2.events().try_exit({m_entities, e1.index()});
    }

    void collider2D::brute_force_coldet(std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
#if defined(PPX_WINDOWS) && !defined(PERF) && !defined(PPX_NO_MULTITHREADING)
        const auto exec = [this, &stchanges](const entity2D &e1)
        {
            for (std::size_t j = 0; j < m_entities->size(); j++)
            {
                collision2D c;
                const entity2D &e2 = (*m_entities)[j];
                if (collide(e1, e2, &c))
                {
                    try_enter_or_stay_callback(e1, e2, c);
                    solve(c, stchanges);
                }
                else
                    try_exit_callback(e1, e2);
            }
        };
        std::for_each(std::execution::par, m_entities->begin(), m_entities->end(), exec);
#else
#ifdef DEBUG
        std::size_t checks = 0, collisions = 0;
#endif
        for (std::size_t i = 0; i < m_entities->size(); i++)
            for (std::size_t j = i + 1; j < m_entities->size(); j++)
            {
#ifdef DEBUG
                checks++;
#endif
                collision2D c;
                const entity2D &e1 = (*m_entities)[i], &e2 = (*m_entities)[j];
                if (collide(e1, e2, &c))
                {
#ifdef DEBUG
                    collisions++;
#endif
                    try_enter_or_stay_callback(e1, e2, c);
                    solve(c, stchanges);
                }
                else
                    try_exit_callback(e1, e2);
            }
        DBG_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for BRUTE FORCE collision detection (QUALITY: {3:.2f}%%)", checks, collisions, checks - collisions, 100.f * (float)m_entities->size() / (float)checks)
#endif
    }

    void collider2D::sort_and_sweep_coldet(std::vector<float> &stchanges)
    {
        PERF_FUNCTION()
#ifdef DEBUG
        std::size_t checks = 0, collisions = 0;
#endif
        std::unordered_set<const entity2D *> eligible;
        sort_intervals();

        eligible.reserve(30);
        for (const interval &itrv : m_intervals)
            if (itrv.type() == interval::LOWER)
            {
                for (const entity2D *e : eligible)
                {
#ifdef DEBUG
                    checks++;
#endif
                    collision2D c;
                    const entity2D &e1 = *e, &e2 = *itrv.entity();
                    if (collide(e1, e2, &c))
                    {
#ifdef DEBUG
                        collisions++;
#endif
                        try_enter_or_stay_callback(e1, e2, c);
                        solve(c, stchanges);
                    }
                    else
                        try_exit_callback(e1, e2);
                }
                eligible.insert(itrv.entity());
            }
            else
                eligible.erase(itrv.entity());
        DBG_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for SORT AND SWEEP collision detection (QUALITY: {3:.2f}%%)", checks, collisions, checks - collisions, 100.f * (float)m_entities->size() / (float)checks)
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

#if defined(PPX_WINDOWS) && !defined(PERF) && !defined(PPX_NO_MULTITHREADING)
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
#ifdef DEBUG
        std::size_t checks = 0, collisions = 0;
#endif
        for (const std::vector<const_entity2D_ptr> *partition : partitions)
            for (std::size_t i = 0; i < partition->size(); i++)
                for (std::size_t j = i + 1; j < partition->size(); j++)
                {
#ifdef DEBUG
                    checks++;
#endif
                    collision2D c;
                    const auto &e1 = (*partition)[i], &e2 = (*partition)[j];
                    if (collide(*e1, *e2, &c))
                    {
#ifdef DEBUG
                        collisions++;
#endif
                        try_enter_or_stay_callback(*e1, *e2, c);
                        solve(c, stchanges);
                    }
                    else
                        try_exit_callback(*e1, *e2);
                }
        DBG_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for QUAD TREE collision detection (QUALITY: {3:.2f}%%)", checks, collisions, checks - collisions, 100.f * (float)m_entities->size() / (float)checks)
#endif
    }

    void collider2D::solve(const collision2D &c,
                           std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        const std::array<float, 6> forces = forces_upon_collision(c);
        for (std::size_t i = 0; i < 3; i++)
        {
            if (c.current->kinematic())
                stchanges[c.current->index() * 6 + i + 3] += forces[i];
            if (c.incoming->kinematic())
                stchanges[c.incoming->index() * 6 + i + 3] += forces[i + 3];
        }
    }

    std::array<float, 6> collider2D::forces_upon_collision(const collision2D &c) const
    {
        PERF_FUNCTION()
        const glm::vec2 rel1 = c.touch1 - c.current->pos(),
                        rel2 = c.touch2 - c.incoming->pos();

        const glm::vec2 vel1 = c.current->vel_at(rel1),
                        vel2 = c.incoming->vel_at(rel2);

        const glm::vec2 force = m_stiffness * (c.touch2 - c.touch1) + m_dampening * (vel2 - vel1);
        const glm::vec2 force1 = c.current->mass() * force, force2 = c.incoming->mass() * force;

        const float torque1 = cross(rel1, force1), torque2 = cross(force2, rel2);
        return {force1.x, force1.y, torque1, -force2.x, -force2.y, torque2};
    }

#ifdef HAS_YAML_CPP
    YAML::Emitter &
    operator<<(YAML::Emitter &out, const collider2D &cld)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "Quad tree" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "Dimensions" << YAML::Value << cld.quad_tree().aabb();
        out << YAML::Key << "Max entities" << YAML::Value << cld.quad_tree().max_entities();
        out << YAML::Key << "Max depth" << YAML::Value << ppx::quad_tree2D::max_depth();
        out << YAML::Key << "Refresh period" << YAML::Value << cld.quad_tree_build_period();
        out << YAML::EndMap;
        out << YAML::Key << "Stiffness" << YAML::Value << cld.stiffness();
        out << YAML::Key << "Dampening" << YAML::Value << cld.dampening();
        out << YAML::Key << "Collision detection" << YAML::Value << cld.coldet();
        out << YAML::Key << "Enabled" << YAML::Value << cld.enabled();
        out << YAML::EndMap;
        return out;
    }
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    Node convert<ppx::collider2D>::encode(const ppx::collider2D &cld)
    {
        Node node;

        Node qt = node["Quad tree"];
        qt["Dimensions"] = cld.quad_tree().aabb();
        qt["Max entities"] = cld.quad_tree().max_entities();
        qt["Max depth"] = ppx::quad_tree2D::max_depth();
        qt["Refresh period"] = cld.quad_tree_build_period();

        node["Stiffness"] = cld.stiffness();
        node["Dampening"] = cld.dampening();
        node["Collision detection"] = (int)cld.coldet();
        node["Enabled"] = cld.enabled();
        return node;
    }
    bool convert<ppx::collider2D>::decode(const Node &node, ppx::collider2D &cld)
    {
        if (!node.IsMap() || node.size() != 5)
            return false;

        const Node &qt = node["Quad tree"];
        cld.quad_tree().aabb(qt["Dimensions"].as<geo::aabb2D>());
        cld.quad_tree().max_entities(qt["Max entities"].as<std::size_t>());
        ppx::quad_tree2D::max_depth(qt["Max depth"].as<std::uint32_t>());
        cld.quad_tree_build_period(qt["Refresh period"].as<std::uint32_t>());

        cld.stiffness(node["Stiffness"].as<float>());
        cld.dampening(node["Dampening"].as<float>());
        cld.coldet((ppx::collider2D::coldet_method)node["Collision detection"].as<int>());
        cld.enabled(node["Enabled"].as<bool>());
        cld.rebuild_quad_tree();

        return true;
    };
}
#endif