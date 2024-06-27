#include "ppx/internal/pch.hpp"
#include "ppx/joints/motor_joint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
motor_joint2D::motor_joint2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.props), vconstraint2D(spc.props), m_force(spc.props.force),
      m_correction_factor(spc.props.correction_factor), m_target_speed(spc.props.target_speed),
      m_target_offset(spc.props.target_offset)
{
}

glm::vec2 motor_joint2D::constraint_velocity() const
{
    glm::vec2 dv = m_body2->meta.ctr_state.velocity - m_body1->meta.ctr_state.velocity;
    if (glm::length2(m_correction) > m_target_speed * m_target_speed)
        return dv + glm::normalize(m_correction) * m_target_speed;
    return dv + m_correction;
}

void motor_joint2D::solve_velocities()
{
    const glm::vec2 impulse = compute_constraint_impulse();
    const glm::vec2 old_impulse = m_cumimpulse;
    const float max_impulse = m_force * world.rk_substep_timestep();

    m_cumimpulse += impulse;
    if (glm::length2(m_cumimpulse) > max_impulse * max_impulse)
        m_cumimpulse = glm::normalize(m_cumimpulse) * max_impulse;

    const glm::vec2 delta_impulse = m_cumimpulse - old_impulse;
    if (!kit::approaches_zero(glm::length2(delta_impulse)))
        apply_linear_impulse(delta_impulse);
}

motor_joint2D::specs::properties motor_joint2D::props() const
{
    specs::properties props;
    fill_cprops(props);
    props.force = m_force;
    props.correction_factor = m_correction_factor;
    props.target_speed = m_target_speed;
    props.target_offset = m_target_offset;
    return props;
}
void motor_joint2D::props(const specs::properties &props)
{
    KIT_ASSERT_ERROR(props.correction_factor >= 0.0f && props.correction_factor <= 1.0f,
                     "Correction factor must be in the range [0, 1]: {0}", props.correction_factor);
    cprops(props);
    m_force = props.force;
    m_correction_factor = props.correction_factor;
    m_target_speed = props.target_speed;
    m_target_offset = props.target_offset;
}

float motor_joint2D::force() const
{
    return m_force;
}
void motor_joint2D::force(const float force)
{
    m_force = force;
    awake();
}

float motor_joint2D::correction_factor() const
{
    return m_correction_factor;
}
void motor_joint2D::correction_factor(const float correction_factor)
{
    m_correction_factor = correction_factor;
    awake();
}

float motor_joint2D::target_speed() const
{
    return m_target_speed;
}
void motor_joint2D::target_speed(const float target_speed)
{
    m_target_speed = target_speed;
    awake();
}

const glm::vec2 &motor_joint2D::target_offset() const
{
    return m_target_offset;
}
void motor_joint2D::target_offset(const glm::vec2 &target_offset)
{
    m_target_offset = target_offset;
    awake();
}

void motor_joint2D::update_constraint_data()
{
    vconstraint2D<2, 0>::update_constraint_data();
    m_correction = m_correction_factor * (m_ganchor2 - m_ganchor1 - m_target_offset) / world.rk_substep_timestep();
}
} // namespace ppx