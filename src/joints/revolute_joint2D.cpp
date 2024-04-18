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
    return m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
           m_body1->ctr_state.velocity_at_centroid_offset(m_offset1);
}

glm::mat2 revolute_joint2D::inverse_mass() const
{
    const glm::vec2 a1 = m_offset1 * m_offset1;
    const glm::vec2 a2 = m_offset2 * m_offset2;
    const glm::vec2 a12 = {m_offset1.x * m_offset1.y, m_offset2.x * m_offset2.y};

    const float im1 = m_body1->props().dynamic.inv_mass;
    const float im2 = m_body2->props().dynamic.inv_mass;

    const float ii1 = m_body1->props().dynamic.inv_inertia;
    const float ii2 = m_body2->props().dynamic.inv_inertia;

    const float cross_term = -ii1 * a12.x - ii2 * a12.y;
    return glm::mat2{{im1 + im2 + ii1 * a1.y + ii2 * a2.y, cross_term},
                     {cross_term, im1 + im2 + ii1 * a1.x + ii2 * a2.x}};
}

} // namespace ppx