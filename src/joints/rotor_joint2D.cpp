#include "ppx/internal/pch.hpp"
#include "ppx/joints/rotor_joint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
rotor_joint2D::rotor_joint2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.props), vconstraint2D(spc.props), m_torque(spc.props.torque),
      m_correction_factor(spc.props.correction_factor), m_target_speed(spc.props.target_speed),
      m_min_angle(spc.props.min_angle), m_max_angle(spc.props.max_angle),
      m_spin_indefinitely(spc.props.spin_indefinitely)
{
}

float rotor_joint2D::constraint_velocity() const
{
    const float dw = state2().angular_velocity - state1().angular_velocity;
    if (m_spin_indefinitely)
        return dw - m_target_speed;
    if (glm::abs(m_correction) > glm::abs(m_target_speed))
        return dw + glm::abs(m_target_speed) * glm::sign(m_correction);
    return dw + m_correction;
}

void rotor_joint2D::solve_velocities()
{
    if (m_legal_angle && !m_spin_indefinitely)
        return;
    const float max_impulse = m_ts * m_torque;
    if (m_spin_indefinitely || kit::approximately(m_max_angle, m_min_angle))
        vconstraint2D<0, 1>::solve_velocities_clamped(-max_impulse, max_impulse);
    else if (m_relangle < m_min_angle)
        vconstraint2D<0, 1>::solve_velocities_clamped(0.f, max_impulse);
    else
        vconstraint2D<0, 1>::solve_velocities_clamped(-max_impulse, 0.f);
}

rotor_joint2D::specs::properties rotor_joint2D::props() const
{
    specs::properties props;
    fill_cprops(props);
    props.torque = m_torque;
    props.correction_factor = m_correction_factor;
    props.target_speed = m_target_speed;
    props.min_angle = m_min_angle;
    props.max_angle = m_max_angle;
    props.spin_indefinitely = m_spin_indefinitely;
    return props;
}
void rotor_joint2D::props(const specs::properties &props)
{
    KIT_ASSERT_ERROR(props.correction_factor >= 0.0f && props.correction_factor <= 1.0f,
                     "Correction factor must be in the range [0, 1]: {0}", props.correction_factor);
    KIT_ASSERT_ERROR(props.min_angle <= props.max_angle, "Min angle must be less than max angle: {0} < {1}",
                     props.min_angle, props.max_angle);
    cprops(props);
    m_torque = props.torque;
    m_correction_factor = props.correction_factor;
    m_target_speed = props.target_speed;
    m_min_angle = props.min_angle;
    m_max_angle = props.max_angle;
    m_spin_indefinitely = props.spin_indefinitely;
}

float rotor_joint2D::torque() const
{
    return m_torque;
}
void rotor_joint2D::torque(const float torque)
{
    m_torque = torque;
    awake();
}

float rotor_joint2D::correction_factor() const
{
    return m_correction_factor;
}
void rotor_joint2D::correction_factor(const float correction_factor)
{
    m_correction_factor = correction_factor;
    awake();
}

float rotor_joint2D::target_speed() const
{
    return m_target_speed;
}
void rotor_joint2D::target_speed(const float target_speed)
{
    m_target_speed = target_speed;
    awake();
}

float rotor_joint2D::min_angle() const
{
    return m_min_angle;
}
void rotor_joint2D::min_angle(const float min_angle)
{
    KIT_ASSERT_ERROR(min_angle <= m_max_angle, "Min angle must be less than max angle: {0} < {1}", min_angle,
                     m_max_angle);
    m_min_angle = min_angle;
    awake();
}

float rotor_joint2D::max_angle() const
{
    return m_max_angle;
}
void rotor_joint2D::max_angle(const float max_angle)
{
    KIT_ASSERT_ERROR(m_min_angle <= max_angle, "Min angle must be less than max angle: {0} < {1}", m_min_angle,
                     max_angle);
    m_max_angle = max_angle;
    awake();
}

bool rotor_joint2D::spin_indefinitely() const
{
    return m_spin_indefinitely;
}
void rotor_joint2D::spin_indefinitely(const bool spin_indefinitely)
{
    m_spin_indefinitely = spin_indefinitely;
    awake();
}

void rotor_joint2D::update_constraint_data()
{
    vconstraint2D<0, 1>::update_constraint_data();
    if (m_spin_indefinitely)
        return;
    const float rot1 = state1().centroid.rotation;
    const float rot2 = state2().centroid.rotation;

    float da = rot2 - rot1;
    da -= glm::round(da / glm::two_pi<float>()) * glm::two_pi<float>();
    m_relangle = da;

    if (da < m_min_angle)
        da = rot2 - rot1 - m_min_angle;
    else if (da > m_max_angle)
        da = rot2 - rot1 - m_max_angle;
    else
    {
        m_correction = 0.f;
        m_legal_angle = true;
        return;
    }

    da -= glm::round(da / glm::two_pi<float>()) * glm::two_pi<float>();
    m_correction = m_correction_factor * da / m_ts;
    m_legal_angle = false;
}
} // namespace ppx