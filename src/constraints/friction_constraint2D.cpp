#include "ppx/internal/pch.hpp"
#include "ppx/constraints/friction_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
friction_constraint2D::friction_constraint2D(const collision2D *collision, const std::size_t manifold_index,
                                             const float friction)
    : constraint2D("Friction"), m_collision(collision),
      m_anchor1(collision->touch1(manifold_index) - collision->body1->position()),
      m_anchor2(collision->touch2(manifold_index) - collision->body2->position()),
      m_tangent(glm::normalize(glm::vec2(-collision->mtv.y, collision->mtv.x))), m_friction(friction)
{
}

float friction_constraint2D::constraint_value() const
{
    return 0.f;
}
float friction_constraint2D::constraint_velocity() const
{
    return glm::dot(m_tangent, m_collision->body1->constraint_velocity_at(m_anchor1) -
                                   m_collision->body2->constraint_velocity_at(m_anchor2));
}

float friction_constraint2D::compute_lambda() const
{
    const float cvel = constraint_velocity();

    const float cross1 = kit::cross2D(m_anchor1, m_tangent);
    const float cross2 = kit::cross2D(m_anchor2, m_tangent);

    const body2D::ptr &body1 = m_collision->body1;
    const body2D::ptr &body2 = m_collision->body2;

    const float inv_mass = body1->inv_mass() + body2->inv_mass() + body1->inv_inertia() * cross1 * cross1 +
                           body2->inv_inertia() * cross2 * cross2;
    return -cvel / inv_mass;
}

void friction_constraint2D::apply_lambda(const float lambda)
{
    const glm::vec2 imp1 = lambda * m_tangent;
    const glm::vec2 imp2 = -imp1;

    const body2D::ptr &body1 = m_collision->body1;
    const body2D::ptr &body2 = m_collision->body2;

    body1->constraint_velocity += body1->inv_mass() * imp1;
    body2->constraint_velocity += body2->inv_mass() * imp2;

    body1->constraint_angular_velocity += body1->inv_inertia() * kit::cross2D(m_anchor1, imp1);
    body2->constraint_angular_velocity += body2->inv_inertia() * kit::cross2D(m_anchor2, imp2);

    body1->apply_simulation_force_at(imp1 / world->integrator.ts.value, m_anchor1);
    body2->apply_simulation_force_at(imp2 / world->integrator.ts.value, m_anchor2);
}

void friction_constraint2D::warmup()
{
    if (kit::approaches_zero(m_accumulated_lambda))
        return;
    m_accumulated_lambda *= world->timestep_ratio();
    apply_lambda(m_accumulated_lambda);
}
void friction_constraint2D::solve()
{
    const float lambda = compute_lambda();
    const float old_lambda = m_accumulated_lambda;
    m_accumulated_lambda = std::clamp(m_accumulated_lambda + lambda, -m_friction * max_lambda, m_friction * max_lambda);

    const float delta_lambda = m_accumulated_lambda - old_lambda;
    if (!kit::approaches_zero(delta_lambda))
        apply_lambda(delta_lambda);
}

void friction_constraint2D::update(const collision2D *collision, const glm::vec2 &normal, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2, float friction)
{
    m_collision = collision;
    m_anchor1 = anchor1;
    m_anchor2 = anchor2;
    m_tangent = glm::vec2(-normal.y, normal.x);
    m_friction = friction;
}

bool friction_constraint2D::contains(kit::uuid id) const
{
    return m_collision->body1->id == id || m_collision->body2->id == id;
}
bool friction_constraint2D::valid() const
{
    return m_collision->body1 && m_collision->body2;
}

} // namespace ppx