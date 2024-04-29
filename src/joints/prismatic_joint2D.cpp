#include "ppx/internal/pch.hpp"
#include "ppx/joints/prismatic_joint2D.hpp"
#include "ppx/body/body2D.hpp"

namespace ppx
{
prismatic_joint2D::prismatic_joint2D(world2D &world, const specs &spc)
    : pvconstraint2D<1, 1>(world, spc, spc.ganchor1, spc.ganchor2),
      m_target_relangle(m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation())
{
    props.axis = spc.deduce_axis ? glm::normalize(spc.ganchor2 - spc.ganchor1) : glm::normalize(props.axis);
}

glm::vec2 prismatic_joint2D::constraint_position() const
{
    KIT_ASSERT_ERROR(kit::approximately(glm::length2(props.axis), 1.f), "The axis must be normalized")
    const float dp = glm::dot(props.axis, m_ganchor2 - m_ganchor1);
    const float da =
        m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation() - m_target_relangle;
    return {dp, da};
}
glm::vec2 prismatic_joint2D::constraint_velocity() const
{
    const float dv = glm::dot(props.axis, m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
                                              m_body1->ctr_state.velocity_at_centroid_offset(m_offset1));
    const float dw = m_body2->ctr_state.angular_velocity - m_body1->ctr_state.angular_velocity;
    return {dv, dw};
}

glm::vec2 prismatic_joint2D::axis_from_angle(const float radians)
{
    return {glm::cos(radians), glm::sin(radians)};
}

} // namespace ppx