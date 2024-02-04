#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/quad_tree.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{

quad_tree::quad_tree(const glm::vec2 &min, const glm::vec2 &max, const std::size_t max_colliders,
                     const std::uint32_t depth)
    : aabb(min, max), m_depth(depth)
{
}

void quad_tree::insert(collider2D *collider)
{
    KIT_ASSERT_CRITICAL(m_colliders.size() <= max_colliders || rock_bottom(),
                        "Quad tree contains more bodies than allowed! - Contained bodies: {0}, maximum bodies: {1}",
                        m_colliders.size(), max_colliders)
    if (!geo::intersects(aabb, collider->shape().bounding_box()))
        return;
    if (full() && !rock_bottom())
        subdivide();
    if (m_partitioned)
        insert_to_children(collider);
    else
        m_colliders.push_back(collider);
}

void quad_tree::collect_partitions(std::vector<const partition *> &partitions) const
{
    if (!m_partitioned)
        partitions.push_back(&m_colliders);
    else
        for (const auto &q : m_children)
            q->collect_partitions(partitions);
}

void quad_tree::clear()
{
    m_partitioned = false;
    m_colliders.clear();
}

void quad_tree::create_children()
{
    m_has_children = true;
    m_partitioned = true;
    const glm::vec2 &mm = aabb.min, &mx = aabb.max;
    const glm::vec2 mid_point = 0.5f * (mm + mx), hdim = 0.5f * (mx - mm);
    m_children[0] = kit::make_scope<quad_tree>(glm::vec2(mm.x, mm.y + hdim.y), glm::vec2(mx.x - hdim.x, mx.y),
                                               max_colliders, m_depth + 1);
    m_children[1] = kit::make_scope<quad_tree>(mid_point, mx, max_colliders, m_depth + 1);
    m_children[2] = kit::make_scope<quad_tree>(mm, mid_point, max_colliders, m_depth + 1);
    m_children[3] = kit::make_scope<quad_tree>(glm::vec2(mm.x + hdim.x, mm.y), glm::vec2(mx.x, mx.y - hdim.y),
                                               max_colliders, m_depth + 1);
}

void quad_tree::reset_children()
{
    m_partitioned = true;
    const glm::vec2 &mm = aabb.min, &mx = aabb.max;
    const glm::vec2 mid_point = 0.5f * (mm + mx), hdim = 0.5f * (mx - mm);
    *(m_children[0]) =
        quad_tree(glm::vec2(mm.x, mm.y + hdim.y), glm::vec2(mx.x - hdim.x, mx.y), max_colliders, m_depth + 1);
    *(m_children[1]) = quad_tree(mid_point, mx, max_colliders, m_depth + 1);
    *(m_children[2]) = quad_tree(mm, mid_point, max_colliders, m_depth + 1);
    *(m_children[3]) =
        quad_tree(glm::vec2(mm.x + hdim.x, mm.y), glm::vec2(mx.x, mx.y - hdim.y), max_colliders, m_depth + 1);
}

void quad_tree::subdivide()
{
    if (m_has_children)
        reset_children();
    else
        create_children();
    for (collider2D *collider : m_colliders)
        insert_to_children(collider);
    m_colliders.clear();
}

void quad_tree::insert_to_children(collider2D *collider)
{
    for (const auto &q : m_children)
        q->insert(collider);
}

bool quad_tree::full() const
{
    return m_colliders.size() >= max_colliders;
}
bool quad_tree::rock_bottom() const
{
    if (m_depth >= max_depth)
        return true;
    const glm::vec2 dim = aabb.dimension();
    return dim.x * dim.y < min_size * min_size;
}

bool quad_tree::partitioned() const
{
    return m_partitioned;
}
const quad_tree::partition &quad_tree::colliders() const
{
    return m_colliders;
}

const std::array<kit::scope<quad_tree>, 4> &quad_tree::children() const
{
    return m_children;
}
const quad_tree &quad_tree::child(std::size_t index) const
{
    KIT_ASSERT_ERROR(index < 4, "Index outside of array bounds. A quad tree can only have 4 children - index: {0}",
                     index)
    return *m_children[index];
}
const quad_tree &quad_tree::operator[](std::size_t index) const
{
    return child(index);
}

} // namespace ppx