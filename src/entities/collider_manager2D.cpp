#include "ppx/internal/pch.hpp"
#include "geo/algorithm/intersection.hpp"
#include "ppx/entities/collider_manager2D.hpp"
#include "ppx/entities/body2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
collider2D &collider_manager2D::add(const body2D::ptr &parent, const collider2D::specs &spc)
{
    const auto it = parent->end();
    const bool empty = parent->empty();

    parent->m_size++;
    collider2D &collider =
        empty ? m_elements.emplace_back(world, parent, spc) : *m_elements.emplace(it, world, parent, spc);

    if (empty)
    {
        collider.index = m_elements.size() - 1;
        parent->m_start = collider.index;
    }
    else
    {
        validate_indices();
        for (body2D &body : world.bodies)
            if (body.m_start > parent->m_start)
                body.m_start++;
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

template <typename Collider, typename C> static std::vector<Collider *> in_area(C &elements, const aabb2D &aabb)
{
    std::vector<Collider *> in_area;
    in_area.reserve(8);

    for (Collider &collider : elements)
        if (geo::intersects(collider.bounding_box(), aabb))
            in_area.emplace_back(&collider);
    return in_area;
}

std::vector<const collider2D *> collider_manager2D::operator[](const aabb2D &aabb) const
{
    return in_area<const collider2D>(m_elements, aabb);
}
std::vector<collider2D *> collider_manager2D::operator[](const aabb2D &aabb)
{
    return in_area<collider2D>(m_elements, aabb);
}

template <typename Collider, typename C> static Collider *at_point(C &elements, const glm::vec2 &point)
{
    for (Collider &collider : elements)
        if (geo::intersects(collider.bounding_box(), point))
            return &collider;
    return nullptr;
}

const collider2D *collider_manager2D::operator[](const glm::vec2 &point) const
{
    return at_point<const collider2D>(m_elements, point);
}
collider2D *collider_manager2D::operator[](const glm::vec2 &point)
{
    return at_point<collider2D>(m_elements, point);
}

bool collider_manager2D::remove(const std::size_t index)
{
    if (index >= m_elements.size())
        return false;
    KIT_INFO("Removing collider with index {0} and id {1}", index, m_elements[index].id)

    events.on_early_removal(m_elements[index]);
    body2D &parent = *m_elements[index].parent(); // Beware: taking the pointer would spurious the index

    if (m_elements.size() - parent.m_start != parent.m_size)
        for (body2D &body : world.bodies)
            if (body.m_start > parent.m_start)
                body.m_start--;
    m_elements.erase(m_elements.begin() + index);
    parent.m_size--;
    parent.update_centroids();
    parent.update_inertia();
    parent.update_colliders();

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
        if (collider.parent())
            collider.mutable_shape().parent(&collider.parent()->centroid_transform());
#ifdef DEBUG
        else
        {
            KIT_ERROR("Collider with id {0} has no parent.", collider.id)
        }
#endif
}

void collider_manager2D::validate()
{
    validate_indices();
    validate_parents();
}

} // namespace ppx