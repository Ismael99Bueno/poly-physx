#include "ppx/internal/pch.hpp"
#include "ppx/joints/weld_joint2D.hpp"
#include "ppx/body/body2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
weld_joint2D::weld_joint2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.ganchor, spc.props), pvconstraint2D(spc.props),
      m_target_relangle(m_body2->state().centroid.rotation - m_body1->state().centroid.rotation)
{
}

glm::vec3 weld_joint2D::constraint_position() const
{
    const glm::vec2 dp = m_ganchor2 - m_ganchor1;
    const float da = state2().centroid.rotation - state1().centroid.rotation - m_target_relangle;
    return {dp, da};
}
glm::vec3 weld_joint2D::constraint_velocity() const
{
    const state2D &st1 = state1();
    const state2D &st2 = state2();
    const glm::vec2 dv = st2.velocity_at_centroid_offset(m_offset2) - st1.velocity_at_centroid_offset(m_offset1);
    const float dw = st2.angular_velocity - st1.angular_velocity;
    return {dv, dw};
}

weld_joint2D::specs::properties weld_joint2D::props() const
{
    return cprops();
}

void weld_joint2D::props(const specs::properties &props)
{
    cprops(props);
}

} // namespace ppx