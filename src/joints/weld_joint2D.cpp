#include "ppx/internal/pch.hpp"
#include "ppx/joints/weld_joint2D.hpp"
#include "ppx/body/body2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
weld_joint2D::weld_joint2D(world2D &world, const specs &spc) : pvconstraint2D<2, 1>(world, spc, spc.ganchor)
{
    if (spc.deduce_angle)
    {
        const float angle = m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation();
        props.min_angle = angle;
        props.max_angle = angle;
    }
}

glm::vec3 weld_joint2D::constraint_position() const
{
    KIT_ASSERT_ERROR(props.min_angle <= props.max_angle, "Invalid angle range")
    const glm::vec2 dp = m_ganchor2 - m_ganchor1;
    if (m_relangle < props.min_angle)
        return {dp, m_relangle - props.min_angle};
    if (m_relangle > props.max_angle)
        return {dp, m_relangle - props.max_angle};
    return {dp, 0.f};
}
glm::vec3 weld_joint2D::constraint_velocity() const
{
    const glm::vec2 dv = m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
                         m_body1->ctr_state.velocity_at_centroid_offset(m_offset1);
    const float dw = m_body2->ctr_state.angular_velocity - m_body1->ctr_state.angular_velocity;
    return {dv, dw};
}

void weld_joint2D::solve_velocities()
{
    if (kit::approximately(props.min_angle, props.max_angle))
        pvconstraint2D<2, 1>::solve_velocities();
    else if (m_relangle < props.min_angle)
        solve_velocities_clamped({-FLT_MAX, -FLT_MAX, 0.f}, {FLT_MAX, FLT_MAX, FLT_MAX});
    else
        solve_velocities_clamped({-FLT_MAX, -FLT_MAX, -FLT_MAX}, {FLT_MAX, FLT_MAX, 0.f});
}

void weld_joint2D::update_constraint_data()
{
    pvconstraint2D<2, 1>::update_constraint_data();
    m_relangle = m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation();
}

} // namespace ppx