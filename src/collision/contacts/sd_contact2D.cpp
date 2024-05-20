#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/sd_contact2D.hpp"

namespace ppx
{
sd_contact2D::sd_contact2D(world2D &world, const collision2D *collision, std::size_t manifold_index)
    : contact_joint2D(world, collision, manifold_index)
{
    compute_parameters();
}

void sd_contact2D::compute_parameters()
{
    m_normal_damping = (1.f - m_collision->restitution) * max_normal_damping;
    m_tangent_damping = m_collision->friction * max_tangent_damping;
}

void sd_contact2D::update(const collision2D *collision, const std::size_t manifold_index)
{
    contact_joint2D::update(collision, manifold_index);
    compute_parameters();
}

void sd_contact2D::solve()
{
    body2D *body1 = m_collider1->body();
    body2D *body2 = m_collider2->body();

    const glm::vec2 &touch1 = m_collision->manifold[m_manifold_index].point;
    const glm::vec2 touch2 = m_collision->manifold[m_manifold_index].point - m_collision->mtv;

    const glm::vec2 offset1 = touch1 - body1->centroid();
    const glm::vec2 offset2 = touch2 - body2->centroid();

    const glm::vec2 relvel =
        (body2->velocity_at_centroid_offset(offset2) - body1->velocity_at_centroid_offset(offset1));

    const glm::vec2 normal_dir = glm::normalize(m_collision->mtv);

    const glm::vec2 normal_vel = glm::dot(normal_dir, relvel) * normal_dir;
    const glm::vec2 tangent_vel = relvel - normal_vel;

    const glm::vec2 force =
        -m_collision->mtv * rigidity + normal_vel * m_normal_damping + tangent_vel * m_tangent_damping;
    const float torque1 = kit::cross2D(offset1, force);
    const float torque2 = kit::cross2D(force, offset2);
    body1->apply_simulation_force(force);
    body2->apply_simulation_force(-force);
    body1->apply_simulation_torque(torque1);
    body2->apply_simulation_torque(torque2);
}
} // namespace ppx