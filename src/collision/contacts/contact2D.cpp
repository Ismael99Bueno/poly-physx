#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/contact2D.hpp"

namespace ppx
{
contact2D::contact2D(const collision2D *collision, std::size_t manifold_index)
    : m_collision(collision), m_manifold_index(manifold_index)
{
}

void contact2D::update(const collision2D *collision, std::size_t manifold_index)
{
    m_collision = collision;
    m_manifold_index = manifold_index;
    m_is_new = false;
    recently_updated = true;
}

const collision2D *contact2D::collision() const
{
    return m_collision;
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