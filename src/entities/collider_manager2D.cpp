#include "ppx/internal/pch.hpp"
#include "geo/algorithm/intersection.hpp"
#include "ppx/entities/collider_manager2D.hpp"
#include "ppx/entities/body2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
collider2D *collider_manager2D::add(body2D *parent, const collider2D::specs &spc)
{
    collider2D *collider = m_allocator.create(parent, spc);
    parent->m_colliders.push_back(collider);

    events.on_addition(collider);
    KIT_INFO("Added collider with index {0}.", m_elements.size())
    m_elements.push_back(collider);
    return collider;
}

template <typename Collider, typename C> static std::vector<Collider *> in_area(C &elements, const aabb2D &aabb)
{
    std::vector<Collider *> in_area;
    in_area.reserve(8);

    for (Collider *collider : elements)
        if (geo::intersects(collider->bounding_box(), aabb))
            in_area.emplace_back(collider);
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
    for (Collider *collider : elements)
        if (geo::intersects(collider->bounding_box(), point))
            return collider;
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

    collider2D *collider = m_elements[index];
    KIT_INFO("Removing collider with index {0}.", index)

    events.on_removal(*collider);
    body2D *parent = collider->body();
    parent->m_colliders.erase(std::find(parent->m_colliders.begin(), parent->m_colliders.end(), collider));

    m_elements.erase(m_elements.begin() + index);
    parent->update_centroids();
    parent->update_inertia();
    parent->update_colliders();

    m_allocator.destroy(collider);
    return true;
}

} // namespace ppx