#include "ppx/internal/pch.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include "ppx/body/body2D.hpp"

namespace ppx
{
revolute_joint2D::revolute_joint2D(world2D &world, const specs &spc) : pvconstraint2D<2, 0>(world, spc, spc.ganchor)
{
}

glm::vec2 revolute_joint2D::constraint_position() const
{
    return m_ganchor2 - m_ganchor1;
}
glm::vec2 revolute_joint2D::constraint_velocity() const
{
    return m_body2->proxy.ctr_state.velocity_at_centroid_offset(m_offset2) -
           m_body1->proxy.ctr_state.velocity_at_centroid_offset(m_offset1);
}

} // namespace ppx