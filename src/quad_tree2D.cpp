#include "quad_tree2D.hpp"
#include "debug.h"

namespace phys
{
    std::uint32_t quad_tree2D::s_max_depth = 4;
    quad_tree2D::quad_tree2D(const alg::vec2 &pos,
                             const alg::vec2 &dim,
                             const std::size_t max_entities,
                             const std::uint32_t depth) : m_pos(pos),
                                                          m_dim(dim),
                                                          m_max_entities(max_entities),
                                                          m_partitioned(false),
                                                          m_has_children(false),
                                                          m_depth(depth)
    {
        m_entities.reserve(max_entities);
    }

    void quad_tree2D::add_if_inside(const const_entity_ptr &e)
    {
        DBG_ASSERT(m_entities.size() <= m_max_entities || rock_bottom(), "Quad tree contains more entities than allowed! - Contained entities: %zu, maximum entities: %zu\n", m_entities.size(), m_max_entities)
        if (!contains(e->bounding_box()))
            return;
        if (full() && !rock_bottom())
            partition();
        if (m_partitioned)
            add_to_children(e);
        else
            m_entities.emplace_back(e);
    }

    void quad_tree2D::partitions(std::vector<const std::vector<const_entity_ptr> *> &partitions) const
    {
        if (!m_partitioned)
            partitions.emplace_back(&m_entities);
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
        const alg::vec2 offset = m_dim / 4.f,
                        inv_offset = {-m_dim.x / 4.f, m_dim.y / 4.f},
                        half_dim = m_dim / 2.f;
        m_children[0] = std::make_unique<quad_tree2D>(m_pos + inv_offset, half_dim, m_max_entities, m_depth + 1);
        m_children[1] = std::make_unique<quad_tree2D>(m_pos + offset, half_dim, m_max_entities, m_depth + 1);
        m_children[2] = std::make_unique<quad_tree2D>(m_pos - offset, half_dim, m_max_entities, m_depth + 1);
        m_children[3] = std::make_unique<quad_tree2D>(m_pos - inv_offset, half_dim, m_max_entities, m_depth + 1);
    }

    void quad_tree2D::partition()
    {
        if (!m_has_children)
            create_children();
        m_partitioned = true;
        for (const const_entity_ptr &e : m_entities)
            add_to_children(e);
        m_entities.clear();
    }

    bool quad_tree2D::contains(const geo::box2D &bbox) const
    {
        const alg::vec2 mm = m_pos - m_dim / 2.f;
        const alg::vec2 mx = m_pos + m_dim / 2.f;
        return geo::box2D::overlap(mm, mx, bbox.min(), bbox.max());
    }

    void quad_tree2D::add_to_children(const const_entity_ptr &e)
    {
        for (const auto &q : m_children)
            q->add_if_inside(e);
    }

    bool quad_tree2D::full() const { return m_entities.size() >= m_max_entities; }
    bool quad_tree2D::rock_bottom() const { return m_depth >= s_max_depth; }

    const alg::vec2 &quad_tree2D::pos() const { return m_pos; }
    const alg::vec2 &quad_tree2D::dim() const { return m_dim; }

    void quad_tree2D::pos(const alg::vec2 &pos) { m_pos = pos; }
    void quad_tree2D::dim(const alg::vec2 &dim) { m_dim = dim; }

    std::size_t quad_tree2D::max_entities() const { return m_max_entities; }
    void quad_tree2D::max_entities(const std::size_t max_entities) { m_max_entities = max_entities; }

    bool quad_tree2D::partitioned() const { return m_partitioned; }
    const std::vector<const_entity_ptr> &quad_tree2D::entities() const { return m_entities; }

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