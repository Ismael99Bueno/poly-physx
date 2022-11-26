#include "quad_tree2D.hpp"

namespace phys
{
    quad_tree2D::quad_tree2D(const alg::vec2 &pos,
                             const alg::vec2 &dim,
                             const std::size_t max_entities) : m_pos(pos),
                                                               m_dim(dim),
                                                               m_max_entities(max_entities),
                                                               m_partitioned(false),
                                                               m_has_children(false)
    {
        m_entities.reserve(max_entities);
    }

    void quad_tree2D::add_if_inside(const const_entity_ptr &e)
    {
        if (!contains(e))
            return;
        if (full())
        {
            if (!m_partitioned)
            {
                if (!m_has_children)
                    create_children();
                partition();
            }
            add_to_children(e);
            return;
        }
        m_entities.emplace_back(e);
    }

    void quad_tree2D::partitions(std::vector<const std::vector<const_entity_ptr> *> &partitions) const
    {
        if (!full() || !m_partitioned)
        {
            partitions.emplace_back(&m_entities);
            return;
        }
        m_top_left->partitions(partitions);
        m_top_right->partitions(partitions);
        m_bottom_left->partitions(partitions);
        m_bottom_right->partitions(partitions);
    }

    void quad_tree2D::clear()
    {
        m_entities.clear();
        if (m_partitioned)
        {
            m_top_left->clear();
            m_top_right->clear();
            m_bottom_left->clear();
            m_bottom_right->clear();
            m_partitioned = false;
        }
    }

    void quad_tree2D::create_children()
    {
        m_has_children = true;
        const alg::vec2 offset = m_dim / 4.f,
                        inv_offset = {-m_dim.x / 4.f, m_dim.y / 4.f},
                        half_dim = m_dim / 2.f;
        m_top_left = std::make_unique<quad_tree2D>(m_pos + inv_offset, half_dim, m_max_entities);
        m_top_right = std::make_unique<quad_tree2D>(m_pos + offset, half_dim, m_max_entities);
        m_bottom_left = std::make_unique<quad_tree2D>(m_pos - offset, half_dim, m_max_entities);
        m_bottom_right = std::make_unique<quad_tree2D>(m_pos - inv_offset, half_dim, m_max_entities);
    }

    void quad_tree2D::partition()
    {
        m_partitioned = true;
        for (const const_entity_ptr &e : m_entities)
            add_to_children(e);
    }

    bool quad_tree2D::contains(const alg::vec2 &p) const
    {
        return 2.f * std::abs(m_pos.x - p.x) <= m_dim.x && 2.f * std::abs(m_pos.y - p.y) <= m_dim.y;
    }

    bool quad_tree2D::contains(const const_entity_ptr &e) const
    {
        for (const alg::vec2 &v : e->shape().vertices())
            if (contains(v))
                return true;
        return false;
    }

    void quad_tree2D::add_to_children(const const_entity_ptr &e)
    {
        m_top_left->add_if_inside(e);
        m_top_right->add_if_inside(e);
        m_bottom_left->add_if_inside(e);
        m_bottom_right->add_if_inside(e);
    }

    bool quad_tree2D::full() const { return m_entities.size() == m_max_entities; }
    const alg::vec2 &quad_tree2D::pos() const { return m_pos; }
    const alg::vec2 &quad_tree2D::dim() const { return m_dim; }
    std::size_t quad_tree2D::max_entities() const { return m_max_entities; }
    bool quad_tree2D::partitioned() const { return m_partitioned; }
    const std::vector<const_entity_ptr> &quad_tree2D::entities() const { return m_entities; }
}