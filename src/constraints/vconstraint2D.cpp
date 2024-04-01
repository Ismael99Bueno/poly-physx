#include "ppx/internal/pch.hpp"
#include "ppx/constraints/vconstraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
float vconstraint2D::compute_impulse() const
{
    return -constraint_velocity() / m_inv_mass;
}

void vconstraint2D::apply_impulse(const float impulse)
{
    const glm::vec2 imp2 = impulse * m_dir;
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
    const float impulse = compute_impulse();
    const float old_impulse = m_cumimpulse;
    m_cumimpulse = std::clamp(m_cumimpulse + impulse, min, max);

    const float delta_impulse = m_cumimpulse - old_impulse;
    if (!kit::approaches_zero(delta_impulse))
        apply_impulse(delta_impulse);
}

void vconstraint2D::solve_unclamped()
{
    const float impulse = compute_impulse();
    m_cumimpulse += impulse;
    apply_impulse(impulse);
}

void vconstraint2D::startup()
{
    m_ganchor1 = m_body1->ctr_state.global_position_point(m_lanchor1);
    if (!m_single_anchor)
        m_ganchor2 = m_body2->ctr_state.global_position_point(m_lanchor2);
    else
        m_ganchor2 = m_ganchor1;

    m_offset1 = m_ganchor1 - m_body1->ctr_state.centroid.position();
    m_offset2 = m_ganchor2 - m_body2->ctr_state.centroid.position();
    m_dir = direction();
    m_inv_mass = inverse_mass();
}

void vconstraint2D::warmup()
{
    if (kit::approaches_zero(m_cumimpulse))
        return;
    apply_impulse(m_cumimpulse);
}

} // namespace ppx