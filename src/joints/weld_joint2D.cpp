#include "ppx/internal/pch.hpp"
#include "ppx/joints/weld_joint2D.hpp"
#include "ppx/body/body2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
weld_joint2D::weld_joint2D(world2D &world, const specs &spc)
    : pvconstraint2D<2, 1>(world, spc, spc.ganchor),
      m_target_relangle(m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation())
{
}

glm::vec3 weld_joint2D::constraint_position() const
{
    const glm::vec2 dp = m_ganchor2 - m_ganchor1;
    const float da =
        m_body2->ctr_state.centroid.rotation() - m_body1->ctr_state.centroid.rotation() - m_target_relangle;
    return {dp, da};
}
glm::vec3 weld_joint2D::constraint_velocity() const
{
    const glm::vec2 dv = m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
                         m_body1->ctr_state.velocity_at_centroid_offset(m_offset1);
    const float dw = m_body2->ctr_state.angular_velocity - m_body1->ctr_state.angular_velocity;
    return {dv, dw};
}

} // namespace ppx