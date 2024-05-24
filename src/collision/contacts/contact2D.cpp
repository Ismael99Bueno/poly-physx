#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/contact2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
contact2D::contact2D(const collision2D *collision, std::size_t manifold_index)
    : joint2D(world, collision->collider1->body(), collision->collider2->body(),
              collision->manifold[manifold_index].point),
      m_collider1(collision->collider1), m_collider2(collision->collider2),
      m_point(collision->manifold[manifold_index].point), m_mtv(collision->mtv),
      m_normal(glm::normalize(collision->mtv)), m_penetration(collision->manifold[manifold_index].penetration),
      m_restitution(collision->restitution), m_friction(collision->friction)
{
}

void contact2D::update(const collision2D *collision, std::size_t manifold_index)
{
    m_collider1 = collision->collider1;
    m_collider2 = collision->collider2;
    m_point = collision->manifold[manifold_index].point;
    m_normal = glm::normalize(collision->mtv);
    m_penetration = collision->manifold[manifold_index].penetration;
    m_restitution = collision->restitution;
    m_friction = collision->friction;
    m_lifetime = 0;
    enabled = true;
}

collider2D *contact2D::collider1() const
{
    return m_collider1;
}

collider2D *contact2D::collider2() const
{
    return m_collider2;
}

const glm::vec2 &contact2D::point() const
{
    return m_point;
}

const glm::vec2 &contact2D::normal() const
{
    return m_normal;
}
float contact2D::penetration() const
{
    return m_penetration;
}
float contact2D::restitution() const
{
    return m_restitution;
}
float contact2D::friction() const
{
    return m_friction;
}

void contact2D::increment_lifetime()
{
    m_lifetime++;
}
bool contact2D::recently_updated() const
{
    return m_lifetime == 0;
}
bool contact2D::expired() const
{
    return m_lifetime >= world.collisions.contacts()->contact_lifetime;
}

} // namespace ppx