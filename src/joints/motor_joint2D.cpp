#include "ppx/internal/pch.hpp"
#include "ppx/joints/motor_joint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
motor_joint2D::motor_joint2D(world2D &world, const specs &spc) : vconstraint2D<2, 0>(world, spc), props(spc.props)
{
}

glm::vec2 motor_joint2D::constraint_velocity() const
{
    KIT_ASSERT_ERROR(props.correction_factor >= 0.0f && props.correction_factor <= 1.0f,
                     "Correction factor must be in the range [0, 1]: {0}", props.correction_factor);
    glm::vec2 dv = m_body2->proxy.ctr_state.velocity - m_body1->proxy.ctr_state.velocity;
    if (glm::length2(m_correction) > props.target_speed * props.target_speed)
        return dv + glm::normalize(m_correction) * props.target_speed;
    return dv + m_correction;
}

void motor_joint2D::solve_velocities()
{
    const glm::vec2 impulse = compute_constraint_impulse();
    const glm::vec2 old_impulse = m_cumimpulse;
    const float max_impulse = props.force * world.rk_substep_timestep();

    m_cumimpulse += impulse;
    if (glm::length2(m_cumimpulse) > max_impulse * max_impulse)
        m_cumimpulse = glm::normalize(m_cumimpulse) * max_impulse;

    const glm::vec2 delta_impulse = m_cumimpulse - old_impulse;
    if (!kit::approaches_zero(glm::length2(delta_impulse)))
        apply_linear_impulse(delta_impulse);
}

void motor_joint2D::update_constraint_data()
{
    vconstraint2D<2, 0>::update_constraint_data();
    m_correction =
        props.correction_factor * (m_ganchor2 - m_ganchor1 - props.target_offset) / world.rk_substep_timestep();
}
} // namespace ppx