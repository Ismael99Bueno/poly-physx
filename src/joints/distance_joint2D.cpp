#include "ppx/internal/pch.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/body/body2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
distance_joint2D::distance_joint2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.ganchor1, spc.ganchor2), props(spc.props),
      m_length(glm::distance(spc.ganchor1, spc.ganchor2))
{
    if (spc.deduce_distance)
    {
        props.min_distance = m_length;
        props.max_distance = m_length;
    }
    m_legal_length = m_length >= props.min_distance && m_length <= props.max_distance;
}

float distance_joint2D::constraint_position() const
{
    KIT_ASSERT_ERROR(props.min_distance <= props.max_distance,
                     "Minimum distance must be less than or equal to maximum distance: {0} <= {1}", props.min_distance,
                     props.max_distance)
    if (m_length < props.min_distance)
        return m_length - props.min_distance;
    if (m_length > props.max_distance)
        return m_length - props.max_distance;
    return 0.f;
}
float distance_joint2D::constraint_velocity() const
{
    return glm::dot(m_dir, m_body2->meta.ctr_state.velocity_at_centroid_offset(m_offset2) -
                               m_body1->meta.ctr_state.velocity_at_centroid_offset(m_offset1));
}

glm::vec2 distance_joint2D::direction() const
{
    const float dst = glm::distance2(m_ganchor1, m_ganchor2);
    return !kit::approaches_zero(dst) ? glm::normalize(m_ganchor2 - m_ganchor1) : glm::vec2(1.f, 0.f);
}

bool distance_joint2D::solve_positions()
{
    if (m_legal_length)
        return true;
    return pvconstraint2D<1, 0>::solve_positions();
}

void distance_joint2D::solve_velocities()
{
    if (m_legal_length)
        return;
    if (kit::approximately(props.min_distance, props.max_distance))
        pvconstraint2D<1, 0>::solve_velocities();
    else if (m_length < props.min_distance)
        solve_velocities_clamped(0.f, FLT_MAX);
    else
        solve_velocities_clamped(-FLT_MAX, 0.f);
}

void distance_joint2D::update_constraint_data()
{
    vconstraint2D<1, 0>::update_constraint_data();
    m_length = glm::distance(m_ganchor1, m_ganchor2);
    m_legal_length = m_length >= props.min_distance && m_length <= props.max_distance;
    m_c = constraint_position();
}
} // namespace ppx