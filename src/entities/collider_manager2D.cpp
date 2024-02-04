#include "ppx/internal/pch.hpp"
#include "geo/algorithm/intersection.hpp"
#include "ppx/entities/collider_manager2D.hpp"
#include "ppx/entities/body2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
collider2D &collider_manager2D::add(const body2D::ptr &parent, const collider2D::specs &spc)
{
    parent->m_size++;
    collider2D &collider = parent->empty() ? m_elements.emplace_back(world, parent, spc)
                                           : *m_elements.emplace(parent->end(), world, parent, spc);
    if (parent->m_size == 1)
        parent->m_start = collider.index;
    else
    {
        for (body2D &body : world.bodies)
            if (body.m_start > parent->m_start)
                body.m_start++;
        validate_indices();
    }
    events.on_addition(collider);
    KIT_INFO("Added collider with index {0} and id {1}.", collider.index, collider.id)
    return collider;
}

collider2D::const_ptr collider_manager2D::ptr(const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    return {&m_elements, index};
}
collider2D::ptr collider_manager2D::ptr(const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    return {&m_elements, index};
}

std::vector<collider2D::const_ptr> collider_manager2D::operator[](const aabb2D &aabb) const
{
    std::vector<collider2D::const_ptr> in_area;
    in_area.reserve(m_elements.size() / 2);

    for (const collider2D &collider : m_elements)
        if (geo::intersects(collider.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_elements, collider.index);
    return in_area;
}
std::vector<collider2D::ptr> collider_manager2D::operator[](const aabb2D &aabb)
{
    std::vector<collider2D::ptr> in_area;
    in_area.reserve(m_elements.size() / 2);

    for (collider2D &collider : m_elements)
        if (geo::intersects(collider.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_elements, collider.index);
    return in_area;
}

const collider2D *collider_manager2D::operator[](const glm::vec2 &point) const
{
    for (const collider2D &collider : m_elements)
        if (geo::intersects(collider.shape().bounding_box(), point))
            return &collider;
    return nullptr;
}
collider2D *collider_manager2D::operator[](const glm::vec2 &point)
{
    for (collider2D &collider : m_elements)
        if (geo::intersects(collider.shape().bounding_box(), point))
            return &collider;
    return nullptr;
}

bool collider_manager2D::remove(const std::size_t index)
{
    if (index >= m_elements.size())
        return false;
    KIT_INFO("Removing collider with id {0}", m_elements[index].id)

    events.on_early_removal(m_elements[index]);
    body2D &parent = m_elements[index].parent();

    if (m_elements.size() - parent.m_start != parent.m_size)
        for (body2D &body : world.bodies)
            if (body.m_start > parent.m_start)
                body.m_start--;
    parent.m_size--;
    parent.update_centroids();
    parent.update_inertia();
    parent.update_colliders();

    m_elements.erase(m_elements.begin() + index);

    validate_indices();

    events.on_late_removal(index);
    return true;
}

void collider_manager2D::validate_indices()
{
    std::size_t index = 0;
    for (auto it = m_elements.begin(); it != m_elements.end(); index++, ++it)
        it->index = index;
}
void collider_manager2D::validate_parents()
{
    for (collider2D &collider : m_elements)
        collider.mutable_shape().parent(&collider.parent().transform());
}

void collider_manager2D::validate()
{
    validate_indices();
    validate_parents();
}

} // namespace ppx