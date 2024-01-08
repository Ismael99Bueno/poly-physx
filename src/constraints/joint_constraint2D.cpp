#include "ppx/internal/pch.hpp"
#include "ppx/constraints/joint_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
joint_constraint2D::joint_constraint2D(const char *name, const bool allow_position_corrections)
    : constraint2D(name), m_allow_position_corrections(allow_position_corrections)
{
}
float joint_constraint2D::compute_lambda(body2D &body1, body2D &body2, const glm::vec2 &anchor1,
                                         const glm::vec2 &anchor2, const glm::vec2 &dir) const
{
    const float cvel = constraint_velocity();

    const float cross1 = kit::cross2D(anchor1, dir);
    const float cross2 = kit::cross2D(anchor2, dir);

    const float inv_mass = body1.inv_mass() + body2.inv_mass() + body1.inv_inertia() * cross1 * cross1 +
                           body2.inv_inertia() * cross2 * cross2;

    if (m_allow_position_corrections && world->constraints.position_corrections)
    {
        const float c = constraint_value();
        static constexpr float stiffness = 1000.f;
        return -(cvel + c * stiffness * world->integrator.ts.value) / inv_mass;
    }
    return -cvel / inv_mass;
}

void joint_constraint2D::apply_warmup(body2D &body1, body2D &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
                                      const glm::vec2 &dir)
{
    if (kit::approaches_zero(m_accumulated_lambda))
        return;
    m_accumulated_lambda *= world->timestep_ratio();
    apply_lambda(m_accumulated_lambda, body1, body2, anchor1, anchor2, dir);
}

void joint_constraint2D::apply_lambda(const float lambda, body2D &body1, body2D &body2, const glm::vec2 &anchor1,
                                      const glm::vec2 &anchor2, const glm::vec2 &dir) const
{
    const glm::vec2 imp2 = lambda * dir;
    const glm::vec2 imp1 = -imp2;

    body1.constraint_velocity += body1.inv_mass() * imp1;
    body2.constraint_velocity += body2.inv_mass() * imp2;

    body1.constraint_angular_velocity += body1.inv_inertia() * kit::cross2D(anchor1, imp1);
    body2.constraint_angular_velocity += body2.inv_inertia() * kit::cross2D(anchor2, imp2);

    body1.apply_simulation_force_at(imp1 / world->integrator.ts.value, anchor1);
    body2.apply_simulation_force_at(imp2 / world->integrator.ts.value, anchor2);
}

void joint_constraint2D::solve_clamped(body2D &body1, body2D &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
                                       const glm::vec2 &dir, float min, float max)
{
    const float lambda = compute_lambda(body1, body2, anchor1, anchor2, dir);
    const float old_lambda = m_accumulated_lambda;
    m_accumulated_lambda = std::clamp(m_accumulated_lambda + lambda, min, max);

    const float delta_lambda = m_accumulated_lambda - old_lambda;
    if (!kit::approaches_zero(delta_lambda))
        apply_lambda(delta_lambda, body1, body2, anchor1, anchor2, dir);
}
void joint_constraint2D::solve_unclamped(body2D &body1, body2D &body2, const glm::vec2 &anchor1,
                                         const glm::vec2 &anchor2, const glm::vec2 &dir)
{
    const float lambda = compute_lambda(body1, body2, anchor1, anchor2, dir);
    m_accumulated_lambda += lambda;
    apply_lambda(lambda, body1, body2, anchor1, anchor2, dir);
}
} // namespace ppx