#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/contact2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
contact2D::contact2D(const collision2D *collision, std::size_t manifold_index)
    : joint2D(world, collision->collider1->body(), collision->collider2->body(),
              collision->manifold[manifold_index].point),
      m_collider1(collision->collider1), m_collider2(collision->collider2),
      m_point(collision->manifold[manifold_index]), m_mtv(collision->mtv), m_normal(glm::normalize(collision->mtv)),
      m_restitution(collision->restitution), m_friction(collision->friction)
{
}

void contact2D::update(const collision2D *collision, std::size_t manifold_index)
{
    m_collider1 = collision->collider1;
    m_collider2 = collision->collider2;
    m_point = collision->manifold[manifold_index];
    m_normal = glm::normalize(collision->mtv);
    m_restitution = collision->restitution;
    m_friction = collision->friction;
    m_age = 0;
    m_enabled = true;
}

collider2D *contact2D::collider1() const
{
    return m_collider1;
}

collider2D *contact2D::collider2() const
{
    return m_collider2;
}

const geo::contact_point2D &contact2D::point() const
{
    return m_point;
}

const glm::vec2 &contact2D::normal() const
{
    return m_normal;
}
float contact2D::restitution() const
{
    return m_restitution;
}
float contact2D::friction() const
{
    return m_friction;
}

void contact2D::on_enter()
{
    m_collider1->events.on_contact_enter(this);
    m_collider2->events.on_contact_enter(this);
    world.collisions.events.on_contact_enter(this);
}
void contact2D::on_exit()
{
    m_collider1->events.on_contact_exit(*this);
    m_collider2->events.on_contact_exit(*this);
    world.collisions.events.on_contact_exit(*this);
}

void contact2D::on_pre_solve()
{
    m_collider1->events.on_contact_pre_solve(this);
    m_collider2->events.on_contact_pre_solve(this);
    world.collisions.events.on_contact_pre_solve(this);
}
void contact2D::on_post_solve()
{
    m_collider1->events.on_contact_post_solve(this);
    m_collider2->events.on_contact_post_solve(this);
    world.collisions.events.on_contact_post_solve(this);
}

bool contact2D::is_contact() const
{
    return true;
}

void contact2D::increment_age()
{
    m_age++;
}
bool contact2D::recently_updated() const
{
    return m_age == 0;
}
bool contact2D::expired() const
{
    return m_age >= world.collisions.contact_solver()->params.contact_lifetime;
}
bool contact2D::asleep() const
{
    return m_body1->asleep() && m_body2->asleep();
}
std::uint32_t contact2D::life_expectancy() const
{
    return world.collisions.contact_solver()->params.contact_lifetime - m_age;
}

} // namespace ppx