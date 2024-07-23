#include "ppx/internal/pch.hpp"
#include "ppx/joints/prismatic_joint.hpp"
#include "ppx/body/body.hpp"

namespace ppx
{
prismatic_joint2D::prismatic_joint2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.ganchor1, spc.ganchor2, spc.props), pvconstraint2D(spc.props), m_axis(spc.props.axis),
      m_target_relangle(m_body2->state().centroid.rotation - m_body1->state().centroid.rotation)
{
    if (spc.deduce_axis)
        m_axis = spc.ganchor2 - spc.ganchor1;
}

glm::vec2 prismatic_joint2D::constraint_position() const
{
    const float dp = glm::dot(m_dir, m_ganchor2 - m_ganchor1);
    const float da = state2().centroid.rotation - state1().centroid.rotation - m_target_relangle;
    return {dp, da};
}
glm::vec2 prismatic_joint2D::constraint_velocity() const
{
    const state2D &st1 = state1();
    const state2D &st2 = state2();
    const float dv =
        glm::dot(m_dir, st1.velocity_at_centroid_offset(m_offset2) - st2.velocity_at_centroid_offset(m_offset1));
    const float dw = st1.angular_velocity - st2.angular_velocity;
    return {dv, dw};
}

prismatic_joint2D::specs::properties prismatic_joint2D::props() const
{
    specs::properties props;
    fill_cprops(props);
    props.axis = m_axis;
    return props;
}
void prismatic_joint2D::props(const specs::properties &props)
{
    KIT_ASSERT_ERROR(!kit::approaches_zero(glm::length2(props.axis)), "Axis must not be zero");
    cprops(props);
    m_axis = props.axis;
}

const glm::vec2 &prismatic_joint2D::axis() const
{
    return m_axis;
}
void prismatic_joint2D::axis(const glm::vec2 &axis)
{
    KIT_ASSERT_ERROR(!kit::approaches_zero(glm::length2(axis)), "Axis must not be zero");
    m_axis = axis;
    awake();
}

glm::vec2 prismatic_joint2D::direction() const
{
    return glm::normalize(glm::vec2(-m_axis.y, m_axis.x));
}

glm::vec2 prismatic_joint2D::axis_from_angle(const float radians)
{
    return {glm::cos(radians), glm::sin(radians)};
}

} // namespace ppx