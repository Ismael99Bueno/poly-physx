#include "ppx/internal/pch.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include "ppx/body/body2D.hpp"

namespace ppx
{
revolute_joint2D::revolute_joint2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.ganchor, spc.props), pvconstraint2D(spc.props)
{
}

glm::vec2 revolute_joint2D::constraint_position() const
{
    return m_ganchor2 - m_ganchor1;
}
glm::vec2 revolute_joint2D::constraint_velocity() const
{
    return state2().velocity_at_centroid_offset(m_offset2) - state1().velocity_at_centroid_offset(m_offset1);
}

revolute_joint2D::specs::properties revolute_joint2D::props() const
{
    return cprops();
}

void revolute_joint2D::props(const specs::properties &props)
{
    cprops(props);
}

} // namespace ppx