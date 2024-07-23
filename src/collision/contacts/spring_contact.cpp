#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/spring_contact.hpp"

namespace ppx
{
spring_contact2D::spring_contact2D(world2D &world, const collision2D *collision, std::size_t manifold_index)
    : joint2D(world, collision->collider1->body(), collision->collider2->body(),
              collision->manifold[manifold_index].point),
      contact2D(collision, manifold_index)
{
    compute_parameters();
}

void spring_contact2D::compute_parameters()
{
    m_normal_damping = (1.f - m_restitution) * max_normal_damping;
    m_tangent_damping = m_friction * max_tangent_damping;
}

void spring_contact2D::update(const collision2D *collision, const std::size_t manifold_index)
{
    contact2D::update(collision, manifold_index);
    compute_parameters();
}

glm::vec3 spring_contact2D::compute_force(const state2D &state1, const state2D &state2) const
{
    const glm::vec2 vel1 = state1.velocity_at_centroid_offset(m_offset1);
    const glm::vec2 vel2 = state2.velocity_at_centroid_offset(m_offset2);

    const glm::vec2 relvel = vel2 - vel1;

    const glm::vec2 normal_vel = glm::dot(m_normal, relvel) * m_normal;
    const glm::vec2 tangent_vel = relvel - normal_vel;

    const glm::vec2 force = m_mtv * rigidity + normal_vel * m_normal_damping - tangent_vel * m_tangent_damping;
    return glm::vec3(force, 0.f);
}
} // namespace ppx