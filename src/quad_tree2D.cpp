#include "ppx/quad_tree2D.hpp"
#include "debug/debug.hpp"
#include "perf/perf.hpp"

namespace ppx
{
    std::uint32_t quad_tree2D::s_max_depth = 4;
    quad_tree2D::quad_tree2D(collider_key,
                             const glm::vec2 &min,
                             const glm::vec2 &max,
                             const std::size_t max_entities,
                             const std::uint32_t depth) : m_aabb(min, max),
                                                          m_max_entities(max_entities),
                                                          m_depth(depth),
                                                          m_partitioned(false),
                                                          m_has_children(false)
    {
        m_entities.reserve(max_entities);
    }

    void quad_tree2D::add_if_inside(const const_entity2D_ptr &e)
    {
        DBG_ASSERT(m_entities.size() <= m_max_entities || rock_bottom(), "Quad tree contains more entities than allowed! - Contained entities: %zu, maximum entities: %zu\n", m_entities.size(), m_max_entities)
        if (!m_aabb.overlaps(e->aabb()))
            return;
        if (full() && !rock_bottom())
            partition();
        if (m_partitioned)
            add_to_children(e);
        else
            m_entities.push_back(e);
    }

    void quad_tree2D::partitions(std::vector<const std::vector<const_entity2D_ptr> *> &partitions) const
    {
        PERF_FUNCTION()
        if (!m_partitioned)
            partitions.push_back(&m_entities);
        else
            for (const auto &q : m_children)
                q->partitions(partitions);
    }

    void quad_tree2D::update(const std::vector<entity2D> &entities)
    {
        clear();
        for (std::size_t i = 0; i < entities.size(); i++)
            add_if_inside({&entities, i});
    }

    void quad_tree2D::rebuild(const std::vector<entity2D> &entities)
    {
        m_has_children = false;
        update(entities);
    }

    void quad_tree2D::clear()
    {
        m_entities.clear();
        if (m_partitioned)
        {
            for (const auto &q : m_children)
                q->clear();
            m_partitioned = false;
        }
    }

    void quad_tree2D::create_children()
    {
        m_has_children = true;
        const glm::vec2 &mm = m_aabb.min(),
                        &mx = m_aabb.max();
        const glm::vec2 mid_point = 0.5f * (mm + mx),
                        hdim = 0.5f * (mx - mm);
        m_children[0] = std::make_unique<quad_tree2D>(collider_key(), glm::vec2(mm.x, mm.y + hdim.y), glm::vec2(mx.x - hdim.x, mx.y), m_max_entities, m_depth + 1);
        m_children[1] = std::make_unique<quad_tree2D>(collider_key(), mid_point, mx, m_max_entities, m_depth + 1);
        m_children[2] = std::make_unique<quad_tree2D>(collider_key(), mm, mid_point, m_max_entities, m_depth + 1);
        m_children[3] = std::make_unique<quad_tree2D>(collider_key(), glm::vec2(mm.x + hdim.x, mm.y), glm::vec2(mx.x, mx.y - hdim.y), m_max_entities, m_depth + 1);
    }

    void quad_tree2D::partition()
    {
        if (!m_has_children)
            create_children();
        m_partitioned = true;
        for (const const_entity2D_ptr &e : m_entities)
            add_to_children(e);
        m_entities.clear();
    }

    void quad_tree2D::add_to_children(const const_entity2D_ptr &e)
    {
        for (const auto &q : m_children)
            q->add_if_inside(e);
    }

    bool quad_tree2D::full() const { return m_entities.size() >= m_max_entities; }
    bool quad_tree2D::rock_bottom() const { return m_depth >= s_max_depth; }

    const geo::aabb2D &quad_tree2D::aabb() const { return m_aabb; }
    void quad_tree2D::aabb(const geo::aabb2D &aabb) { m_aabb = aabb; }

    std::size_t quad_tree2D::max_entities() const { return m_max_entities; }
    void quad_tree2D::max_entities(const std::size_t max_entities) { m_max_entities = max_entities; }

    bool quad_tree2D::partitioned() const { return m_partitioned; }
    const std::vector<const_entity2D_ptr> &quad_tree2D::entities() const { return m_entities; }

    const std::array<std::unique_ptr<quad_tree2D>, 4> &quad_tree2D::children() const { return m_children; }
    const quad_tree2D &quad_tree2D::child(std::size_t index) const
    {
        DBG_ASSERT(index < 4, "Index outside of array bounds. A quad tree can only have 4 children - index: %zu\n", index)
        return *m_children[index];
    }
    const quad_tree2D &quad_tree2D::operator[](std::size_t index) const { return child(index); }

    std::uint32_t quad_tree2D::max_depth() { return s_max_depth; }
    void quad_tree2D::max_depth(std::uint32_t max_depth) { s_max_depth = max_depth; }
}