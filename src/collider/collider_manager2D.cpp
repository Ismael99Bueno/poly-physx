#include "ppx/internal/pch.hpp"
#include "ppx/collider/collider_manager2D.hpp"
#include "ppx/collision/broad/quad_tree_broad2D.hpp"
#include "ppx/body/body2D.hpp"
#include "ppx/world2D.hpp"
#include "geo/algorithm/intersection2D.hpp"
#include "kit/container/quad_tree.hpp"

namespace ppx
{
collider2D *collider_manager2D::add(body2D *parent, const collider2D::specs &spc)
{
    collider2D *collider = allocator<collider2D>::create(world, parent, spc);
    m_elements.push_back(collider);

    parent->m_colliders.push_back(collider);
    collider->body()->full_update();

    if (auto qt = world.collisions.broad<quad_tree_broad2D>())
        qt->insert(collider);

    events.on_addition(collider);
    KIT_INFO("Added collider with index {0}.", m_elements.size() - 1)

    return collider;
}

static bool cast_check(collider2D *collider, const ray2D &ray, ray2D::hit<collider2D> &closest)
{
    if (!geo::intersects(collider->bounding_box(), ray))
        return false;
    ray2D::hit<collider2D> hit;
    if (auto poly = collider->shape_if<polygon>())
        hit = geo::intersects(*poly, ray, collider);
    else
        hit = geo::intersects(collider->shape<circle>(), ray, collider);
    if (hit)
    {
        closest = hit;
        return true;
    }
    return false;
}

static void cast_qt_recursive(const quad_tree::node &qtnode, ray2D &ray, ray2D::hit<collider2D> &closest)
{
    if (!geo::intersects(qtnode.aabb, ray))
        return;
    for (const qt_element &qtelm : qtnode.elements)
        if (cast_check(qtelm.collider, ray, closest))
            ray.resize(closest.distance);
    if (qtnode.partitioned())
        for (auto child : qtnode.children)
            cast_qt_recursive(*child, ray, closest);
}

ray2D::hit<collider2D> collider_manager2D::cast(ray2D ray) const
{
    ray2D::hit<collider2D> closest;
    closest.distance = FLT_MAX;
    const auto qtbroad = world.collisions.broad<quad_tree_broad2D>();
    if (!qtbroad)
    {
        for (collider2D *collider : m_elements)
            if (cast_check(collider, ray, closest))
                ray.resize(closest.distance);
    }
    else
        cast_qt_recursive(qtbroad->quad_tree().root(), ray, closest);
    return closest;
}

template <typename Collider>
static void in_area_qt_recursive(const quad_tree::node &qtnode, std::vector<Collider *> &in_area, const aabb2D &aabb,
                                 const polygon &aabb_poly)
{
    if (!geo::intersects(qtnode.aabb, aabb))
        return;
    for (const qt_element &qtelm : qtnode.elements)
        if (geo::intersects(qtelm(), aabb) && geo::gjk(qtelm.collider->shape(), aabb_poly))
            in_area.push_back(qtelm.collider);
    if (qtnode.partitioned())
        for (auto child : qtnode.children)
            in_area_qt_recursive(*child, in_area, aabb, aabb_poly);
}

template <typename Collider, typename C>
static std::vector<Collider *> in_area(const world2D &world, C &elements, const aabb2D &aabb)
{
    std::vector<Collider *> in_area;
    if (kit::approaches_zero(aabb.area()))
        return in_area;
    in_area.reserve(8);

    const glm::vec2 tr = aabb.max;
    const glm::vec2 bl = aabb.min;
    const glm::vec2 tl = {bl.x, tr.y};
    const glm::vec2 br = {tr.x, bl.y};
    const polygon aabb_poly{bl, br, tr, tl};

    const auto qtbroad = world.collisions.broad<quad_tree_broad2D>();
    if (!qtbroad)
    {
        for (Collider *collider : elements)
            if (geo::intersects(collider->bounding_box(), aabb) && geo::gjk(collider->shape(), aabb_poly))
                in_area.emplace_back(collider);
    }
    else
        in_area_qt_recursive(qtbroad->quad_tree().root(), in_area, aabb, aabb_poly);
    return in_area;
}

std::vector<const collider2D *> collider_manager2D::operator[](const aabb2D &aabb) const
{
    return in_area<const collider2D>(world, m_elements, aabb);
}
std::vector<collider2D *> collider_manager2D::operator[](const aabb2D &aabb)
{
    return in_area<collider2D>(world, m_elements, aabb);
}

template <typename Collider, typename C> static Collider *at_point(C &elements, const glm::vec2 &point)
{
    for (Collider *collider : elements)
        if (geo::intersects(collider->bounding_box(), point) && collider->shape().contains_point(point))
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
    world.collisions.contact_solver()->remove_any_contacts_with(collider);

    if (auto qt = world.collisions.broad<quad_tree_broad2D>())
        qt->erase(collider);

    body2D *parent = collider->body();
    parent->m_colliders.erase(std::find(parent->m_colliders.begin(), parent->m_colliders.end(), collider));

    if (index != m_elements.size() - 1)
    {
        m_elements[index] = m_elements.back();
        m_elements[index]->index = index;
    }
    m_elements.pop_back();
    parent->full_update();

    allocator<collider2D>::destroy(collider);
    return true;
}

} // namespace ppx