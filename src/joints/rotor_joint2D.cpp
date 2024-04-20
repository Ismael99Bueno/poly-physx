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
    const float dw = m_body2->ctr_state.angular_velocity - m_body1->ctr_state.angular_velocity;
    if (props.spin_indefinitely)
        return dw - props.target_speed;
    return dw + m_correction;
}

void rotor_joint2D::solve_velocities()
{
    const float max_impulse = world.rk_substep_timestep() * props.torque;
    vconstraint2D<0, 1>::solve_velocities_clamped(-max_impulse, max_impulse);
}

void rotor_joint2D::update_constraint_data()
{
    vconstraint2D<0, 1>::update_constraint_data();
    float da = m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation() - props.target_offset;
    da -= glm::round(da / glm::two_pi<float>()) * glm::two_pi<float>();
    m_correction = props.correction_factor * da / world.rk_substep_timestep();
}
} // namespace ppx