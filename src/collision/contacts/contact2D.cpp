#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/contact2D.hpp"

namespace ppx
{
contact2D::contact2D(const collision2D *collision, std::size_t manifold_index)
    : m_collision(collision), m_collider1(collision->collider1), m_collider2(collision->collider2),
      m_manifold_index(manifold_index)
{
}

void contact2D::update(const collision2D *collision, std::size_t manifold_index)
{
    m_collision = collision;
    m_collider1 = collision->collider1;
    m_collider2 = collision->collider2;
    m_manifold_index = manifold_index;
    m_is_new = false;
    recently_updated = true;
}

const collision2D *contact2D::collision() const
{
    return m_collision;
}

const collider2D *contact2D::collider1() const
{
    return m_collider1;
}

const collider2D *contact2D::collider2() const
{
    return m_collider2;
}

collider2D *contact2D::collider1()
{
    return m_collider1;
}

collider2D *contact2D::collider2()
{
    return m_collider2;
}

std::size_t contact2D::manifold_index() const
{
    return m_manifold_index;
}

bool contact2D::is_new() const
{
    return m_is_new;
}

} // namespace ppx