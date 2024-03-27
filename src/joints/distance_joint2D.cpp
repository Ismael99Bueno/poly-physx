#include "ppx/internal/pch.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
distance_joint2D::distance_joint2D(world2D &world, const specs &spc)
    : pvconstraint2D(world, world.bodies[spc.bindex1], world.bodies[spc.bindex2], spc.ganchor1, spc.ganchor2)
{
    if (kit::approximately(spc.min_distance, spc.max_distance))
    {
        const float length = glm::distance(ganchor1(), ganchor2());
        min_distance = length;
        max_distance = length;
    }
}

float distance_joint2D::constraint_position() const
{
    KIT_ASSERT_ERROR(min_distance <= max_distance,
                     "Minimum distance must be less than or equal to maximum distance: {0} <= {1}", min_distance,
                     max_distance)
    const glm::vec2 ganchor1 = m_body1->ctr_state.global_position_point(m_lanchor1);
    const glm::vec2 ganchor2 = m_body2->ctr_state.global_position_point(m_lanchor2);
    const float length = glm::distance(ganchor1, ganchor2);

    if (length < min_distance)
        return length - min_distance;
    if (length > max_distance)
        return length - max_distance;
    return 0.f;
}
float distance_joint2D::constraint_velocity() const
{
    return glm::dot(m_dir, m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
                               m_body1->ctr_state.velocity_at_centroid_offset(m_offset1));
}

float distance_joint2D::inverse_mass() const
{
    const float cross1 = kit::cross2D(m_offset1, m_dir);
    const float cross2 = kit::cross2D(m_offset2, m_dir);
    return m_body1->props().dynamic.inv_mass + m_body2->props().dynamic.inv_mass +
           m_body1->props().dynamic.inv_inertia * cross1 * cross1 +
           m_body2->props().dynamic.inv_inertia * cross2 * cross2;
}

glm::vec2 distance_joint2D::direction() const
{
    return glm::normalize(m_ganchor2 - m_ganchor1);
}

void distance_joint2D::solve()
{
    if (legal_length())
        return;
    if (kit::approximately(min_distance, max_distance))
        solve_unclamped();
    else if (m_length < min_distance)
        solve_clamped(0.f, FLT_MAX);
    else
        solve_clamped(-FLT_MAX, 0.f);
}

bool distance_joint2D::adjust_positions()
{
    return adjust_unclamped();
}

bool distance_joint2D::legal_length() const
{
    return m_length >= min_distance && m_length <= max_distance;
}

void distance_joint2D::startup()
{
    pvconstraint2D::startup();
    m_length = glm::distance(m_ganchor1, m_ganchor2);
}
} // namespace ppx