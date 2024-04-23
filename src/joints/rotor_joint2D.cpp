#include "ppx/internal/pch.hpp"
#include "ppx/joints/rotor_joint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
rotor_joint2D::rotor_joint2D(world2D &world, const specs &spc) : vconstraint2D<0, 1>(world, spc), props(spc.props)
{
}

float rotor_joint2D::constraint_velocity() const
{
    KIT_ASSERT_ERROR(props.correction_factor >= 0.0f && props.correction_factor <= 1.0f,
                     "Correction factor must be in the range [0, 1]: {0}", props.correction_factor);
    KIT_ASSERT_ERROR(props.min_offset <= props.max_offset, "Min offset must be less than max offset: {0} < {1}",
                     props.min_offset, props.max_offset);
    const float dw = m_body2->ctr_state.angular_velocity - m_body1->ctr_state.angular_velocity;
    if (props.spin_indefinitely)
        return dw - props.target_speed;
    if (glm::abs(m_correction) > glm::abs(props.target_speed))
        return dw + glm::abs(props.target_speed) * glm::sign(m_correction);
    return dw + m_correction;
}

void rotor_joint2D::solve_velocities()
{
    if (m_legal_offset && !props.spin_indefinitely)
        return;
    const float max_impulse = world.rk_substep_timestep() * props.torque;
    if (props.spin_indefinitely || kit::approximately(props.max_offset, props.min_offset))
        vconstraint2D<0, 1>::solve_velocities_clamped(-max_impulse, max_impulse);
    else if (m_relangle < props.min_offset)
        vconstraint2D<0, 1>::solve_velocities_clamped(0.f, max_impulse);
    else
        vconstraint2D<0, 1>::solve_velocities_clamped(-max_impulse, 0.f);
}

void rotor_joint2D::update_constraint_data()
{
    vconstraint2D<0, 1>::update_constraint_data();
    if (props.spin_indefinitely)
        return;
    float da = m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation();
    da -= glm::round(da / glm::two_pi<float>()) * glm::two_pi<float>();
    m_relangle = da;

    if (da < props.min_offset)
        da = m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation() - props.min_offset;
    else if (da > props.max_offset)
        da = m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation() - props.max_offset;
    else
    {
        m_correction = 0.f;
        m_legal_offset = true;
        return;
    }

    da -= glm::round(da / glm::two_pi<float>()) * glm::two_pi<float>();
    m_correction = props.correction_factor * da / world.rk_substep_timestep();
    m_legal_offset = false;
}
} // namespace ppx