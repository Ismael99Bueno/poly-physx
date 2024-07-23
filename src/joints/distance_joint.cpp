#include "ppx/internal/pch.hpp"
#include "ppx/joints/distance_joint.hpp"
#include "ppx/body/body.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
distance_joint2D::distance_joint2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.ganchor1, spc.ganchor2, spc.props), pvconstraint2D{spc.props},
      m_min_distance(spc.props.min_distance), m_max_distance(spc.props.max_distance),
      m_length(glm::distance(spc.ganchor1, spc.ganchor2))
{
    if (spc.deduce_distance)
    {
        m_min_distance = m_length;
        m_max_distance = m_length;
    }
    m_legal_length = m_length >= m_min_distance && m_length <= m_max_distance;
}

float distance_joint2D::constraint_position() const
{
    if (m_length < m_min_distance)
        return m_length - m_min_distance;
    if (m_length > m_max_distance)
        return m_length - m_max_distance;
    return 0.f;
}
float distance_joint2D::constraint_velocity() const
{
    return glm::dot(m_dir,
                    state2().velocity_at_centroid_offset(m_offset2) - state1().velocity_at_centroid_offset(m_offset1));
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
    if (kit::approximately(m_min_distance, m_max_distance))
        pvconstraint2D<1, 0>::solve_velocities();
    else if (m_length < m_min_distance)
        solve_velocities_clamped(0.f, FLT_MAX);
    else
        solve_velocities_clamped(-FLT_MAX, 0.f);
}

distance_joint2D::specs::properties distance_joint2D::props() const
{
    specs::properties props;
    fill_cprops(props);
    props.min_distance = m_min_distance;
    props.max_distance = m_max_distance;
    return props;
}
void distance_joint2D::props(const specs::properties &props)
{
    KIT_ASSERT_ERROR(props.min_distance <= props.max_distance,
                     "Minimum distance must be less than or equal to maximum distance: {0} <= {1}", props.min_distance,
                     props.max_distance)
    cprops(props);
    m_min_distance = props.min_distance;
    m_max_distance = props.max_distance;
}

float distance_joint2D::min_distance() const
{
    return m_min_distance;
}
void distance_joint2D::min_distance(const float min_distance)
{
    KIT_ASSERT_ERROR(min_distance <= m_max_distance,
                     "Minimum distance must be less than or equal to maximum distance: {0} <= {1}", min_distance,
                     m_max_distance);
    m_min_distance = min_distance;
    awake();
}

float distance_joint2D::max_distance() const
{
    return m_max_distance;
}
void distance_joint2D::max_distance(const float max_distance)
{
    KIT_ASSERT_ERROR(max_distance >= m_min_distance,
                     "Maximum distance must be greater than or equal to minimum distance: {0} >= {1}", max_distance,
                     m_min_distance);
    m_max_distance = max_distance;
    awake();
}

void distance_joint2D::update_constraint_data()
{
    vconstraint2D<1, 0>::update_constraint_data();
    m_length = glm::distance(m_ganchor1, m_ganchor2);
    m_legal_length = m_length >= m_min_distance && m_length <= m_max_distance;
    m_c = constraint_position();
}
} // namespace ppx