#include "ppx/internal/pch.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
contact_constraint2D::contact_constraint2D(const collision2D *collision, const std::size_t manifold_index)
    : constraint2D("Contact"), m_collision(collision),
      m_anchor1(collision->touch1(manifold_index) - collision->body1->position()),
      m_anchor2(collision->touch2(manifold_index) - collision->body2->position()),
      m_normal(glm::normalize(collision->mtv)), m_index(manifold_index)
{
}

float contact_constraint2D::constraint_value() const
{
    return glm::dot(m_collision->touch1(m_index) - m_collision->touch2(m_index), m_normal);
}
float contact_constraint2D::constraint_velocity() const
{
    return glm::dot(m_normal, m_collision->body1->constraint_velocity_at(m_anchor1) -
                                  m_collision->body2->constraint_velocity_at(m_anchor2));
}

float contact_constraint2D::compute_lambda() const
{
    const float cvel = constraint_velocity();

    const float cross1 = kit::cross2D(m_anchor1, m_normal);
    const float cross2 = kit::cross2D(m_anchor2, m_normal);

    const body2D::ptr &body1 = m_collision->body1;
    const body2D::ptr &body2 = m_collision->body2;

    const float inv_mass = body1->inv_mass() + body2->inv_mass() + body1->inv_inertia() * cross1 * cross1 +
                           body2->inv_inertia() * cross2 * cross2;

    if (world->constraints.position_corrections)
    {
        const float c = constraint_value();
        static constexpr float stiffness = 1000.f;
        return -(cvel + c * stiffness * world->integrator.ts.value) / inv_mass;
    }
    return -cvel / inv_mass;
}

void contact_constraint2D::apply_lambda(const float lambda)
{
    const glm::vec2 imp1 = lambda * m_normal;
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

void contact_constraint2D::warmup()
{
    if (kit::approaches_zero(m_accumulated_lambda))
        return;
    m_accumulated_lambda *= world->timestep_ratio();
    apply_lambda(m_accumulated_lambda);
}
void contact_constraint2D::solve()
{
    const float lambda = compute_lambda();
    const float old_lambda = m_accumulated_lambda;

    m_accumulated_lambda += lambda;
    m_accumulated_lambda = std::min(0.f, m_accumulated_lambda);

    const float delta_lambda = m_accumulated_lambda - old_lambda;
    if (!kit::approaches_zero(delta_lambda))
        apply_lambda(delta_lambda);
}

bool contact_constraint2D::contains(kit::uuid id) const
{
    return m_collision->body1->id == id || m_collision->body2->id == id;
}

bool contact_constraint2D::valid() const
{
    return m_collision->body1 && m_collision->body2;
}

} // namespace ppx
