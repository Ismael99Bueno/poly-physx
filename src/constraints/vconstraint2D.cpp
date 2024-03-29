#include "ppx/internal/pch.hpp"
#include "ppx/constraints/vconstraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
float vconstraint2D::compute_velocity_lambda() const
{
    return -constraint_velocity() / m_inv_mass;
}

void vconstraint2D::apply_velocity_lambda(const float lambda)
{
    const glm::vec2 imp2 = lambda * m_dir;
    const glm::vec2 imp1 = -imp2;

    m_body1->ctr_state.velocity += m_body1->props().dynamic.inv_mass * imp1;
    m_body2->ctr_state.velocity += m_body2->props().dynamic.inv_mass * imp2;

    m_body1->ctr_state.angular_velocity += m_body1->props().dynamic.inv_inertia * kit::cross2D(m_offset1, imp1);
    m_body2->ctr_state.angular_velocity += m_body2->props().dynamic.inv_inertia * kit::cross2D(m_offset2, imp2);

    const glm::vec2 f1 = imp1 / world.rk_substep_timestep();
    const glm::vec2 f2 = imp2 / world.rk_substep_timestep();
    const float torque1 = kit::cross2D(m_offset1, f1);
    const float torque2 = kit::cross2D(m_offset2, f2);

    m_body1->apply_simulation_force(f1);
    m_body1->apply_simulation_torque(torque1);
    m_body2->apply_simulation_force(f2);
    m_body2->apply_simulation_torque(torque2);
}

void vconstraint2D::solve_clamped(const float min, const float max)
{
    const float lambda = compute_velocity_lambda();
    const float old_lambda = m_cumlambda;
    m_cumlambda = std::clamp(m_cumlambda + lambda, min, max);

    const float delta_lambda = m_cumlambda - old_lambda;
    if (!kit::approaches_zero(delta_lambda))
        apply_velocity_lambda(delta_lambda);
}

void vconstraint2D::solve_unclamped()
{
    const float lambda = compute_velocity_lambda();
    m_cumlambda += lambda;
    apply_velocity_lambda(lambda);
}

void vconstraint2D::startup()
{
    m_ganchor1 = m_body1->ctr_state.global_position_point(m_lanchor1);
    m_ganchor2 = m_body2->ctr_state.global_position_point(m_lanchor2);
    m_offset1 = m_ganchor1 - m_body1->ctr_state.centroid.position();
    m_offset2 = m_ganchor2 - m_body2->ctr_state.centroid.position();
    m_dir = direction();
    m_inv_mass = inverse_mass();
}

void vconstraint2D::warmup()
{
    if (kit::approaches_zero(m_cumlambda))
        return;
    m_cumlambda *= world.timestep_ratio();
    apply_velocity_lambda(m_cumlambda);
}

} // namespace ppx