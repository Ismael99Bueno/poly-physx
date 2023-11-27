#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/quad_tree2D.hpp"
#include "geo/intersection.hpp"
#include "kit/profile/perf.hpp"

namespace ppx
{

quad_tree2D::quad_tree2D(const glm::vec2 &min, const glm::vec2 &max, const std::size_t max_bodies,
                         const std::uint32_t depth)
    : aabb(min, max), m_depth(depth)
{
    m_bodies.reserve(4 * max_bodies);
}

void quad_tree2D::insert(const body2D *body)
{
    KIT_ASSERT_CRITICAL(m_bodies.size() <= max_bodies || rock_bottom(),
                        "Quad tree contains more bodies than allowed! - Contained bodies: {0}, maximum bodies: {1}",
                        m_bodies.size(), max_bodies)
    if (!geo::intersects(aabb, body->shape().bounding_box()))
        return;
    if (full() && !rock_bottom())
        subdivide();
    if (m_partitioned)
        insert_to_children(body);
    else
        m_bodies.push_back(body);
}

void quad_tree2D::collect_partitions(std::vector<const partition *> &partitions) const
{
    if (!m_partitioned)
        partitions.push_back(&m_bodies);
    else
        for (const auto &q : m_children)
            q->collect_partitions(partitions);
}

void quad_tree2D::clear()
{
    m_partitioned = false;
    m_bodies.clear();
}

void quad_tree2D::create_children()
{
    m_has_children = true;
    m_partitioned = true;
    const glm::vec2 &mm = aabb.min(), &mx = aabb.max();
    const glm::vec2 mid_point = 0.5f * (mm + mx), hdim = 0.5f * (mx - mm);
    m_children[0] = kit::make_scope<quad_tree2D>(glm::vec2(mm.x, mm.y + hdim.y), glm::vec2(mx.x - hdim.x, mx.y),
                                                 max_bodies, m_depth + 1);
    m_children[1] = kit::make_scope<quad_tree2D>(mid_point, mx, max_bodies, m_depth + 1);
    m_children[2] = kit::make_scope<quad_tree2D>(mm, mid_point, max_bodies, m_depth + 1);
    m_children[3] = kit::make_scope<quad_tree2D>(glm::vec2(mm.x + hdim.x, mm.y), glm::vec2(mx.x, mx.y - hdim.y),
                                                 max_bodies, m_depth + 1);
}

void quad_tree2D::reset_children()
{
    m_partitioned = true;
    const glm::vec2 &mm = aabb.min(), &mx = aabb.max();
    const glm::vec2 mid_point = 0.5f * (mm + mx), hdim = 0.5f * (mx - mm);
    *(m_children[0]) =
        quad_tree2D(glm::vec2(mm.x, mm.y + hdim.y), glm::vec2(mx.x - hdim.x, mx.y), max_bodies, m_depth + 1);
    *(m_children[1]) = quad_tree2D(mid_point, mx, max_bodies, m_depth + 1);
    *(m_children[2]) = quad_tree2D(mm, mid_point, max_bodies, m_depth + 1);
    *(m_children[3]) =
        quad_tree2D(glm::vec2(mm.x + hdim.x, mm.y), glm::vec2(mx.x, mx.y - hdim.y), max_bodies, m_depth + 1);
}

void quad_tree2D::subdivide()
{
    if (m_has_children)
        reset_children();
    else
        create_children();
    for (const body2D *body : m_bodies)
        insert_to_children(body);
    m_bodies.clear();
}

void quad_tree2D::insert_to_children(const body2D *body)
{
    for (const auto &q : m_children)
        q->insert(body);
}

bool quad_tree2D::full() const
{
    return m_bodies.size() >= max_bodies;
}
bool quad_tree2D::rock_bottom() const
{
    if (m_depth >= max_depth)
        return true;
    const glm::vec2 diff = aabb.max() - aabb.min();
    return diff.x * diff.y < min_size * min_size;
}

bool quad_tree2D::partitioned() const
{
    return m_partitioned;
}
const quad_tree2D::partition &quad_tree2D::bodies() const
{
    return m_bodies;
}

const std::array<kit::scope<quad_tree2D>, 4> &quad_tree2D::children() const
{
    return m_children;
}
const quad_tree2D &quad_tree2D::child(std::size_t index) const
{
    KIT_ASSERT_ERROR(index < 4, "Index outside of array bounds. A quad tree can only have 4 children - index: {0}",
                     index)
    return *m_children[index];
}
const quad_tree2D &quad_tree2D::operator[](std::size_t index) const
{
    return child(index);
}

} // namespace ppx