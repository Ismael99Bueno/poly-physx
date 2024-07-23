#include "ppx/internal/pch.hpp"
#include "ppx/joints/ball_joint.hpp"
#include "ppx/body/body.hpp"

namespace ppx
{
ball_joint2D::ball_joint2D(world2D &world, const specs &spc)
    : joint2D{world, spc, spc.props}, pvconstraint2D{spc.props}, m_min_angle(spc.props.min_angle),
      m_max_angle(spc.props.max_angle)
{
    KIT_ASSERT_ERROR(m_min_angle <= m_max_angle, "Min angle must be less than max angle: {0} < {1}", m_min_angle,
                     m_max_angle);
    if (spc.deduce_angle)
    {
        m_relangle = m_body2->state().centroid.rotation - m_body1->state().centroid.rotation;
        m_min_angle = m_relangle;
        m_max_angle = m_relangle;
    }
}

float ball_joint2D::constraint_position() const
{
    if (m_relangle < m_min_angle)
        return m_relangle - m_min_angle;
    if (m_relangle > m_max_angle)
        return m_relangle - m_max_angle;
    return 0.f;
}

float ball_joint2D::constraint_velocity() const
{
    return state2().angular_velocity - state1().angular_velocity;
}

bool ball_joint2D::solve_positions()
{
    if (m_legal_angle)
        return true;
    return pvconstraint2D<0, 1>::solve_positions();
}

void ball_joint2D::solve_velocities()
{
    if (m_legal_angle)
        return;
    if (kit::approximately(m_min_angle, m_max_angle))
        pvconstraint2D<0, 1>::solve_velocities();
    else if (m_relangle < m_min_angle)
        solve_velocities_clamped(0.f, FLT_MAX);
    else
        solve_velocities_clamped(-FLT_MAX, 0.f);
}

ball_joint2D::specs::properties ball_joint2D::props() const
{
    specs::properties props;
    fill_cprops(props);
    props.min_angle = m_min_angle;
    props.max_angle = m_max_angle;
    return props;
}
void ball_joint2D::props(const specs::properties &props)
{
    KIT_ASSERT_ERROR(props.min_angle <= props.max_angle, "Min angle must be less than max angle: {0} < {1}",
                     props.min_angle, props.max_angle);
    cprops(props);
    m_min_angle = props.min_angle;
    m_max_angle = props.max_angle;
}

float ball_joint2D::min_angle() const
{
    return m_min_angle;
}
void ball_joint2D::min_angle(const float min_angle)
{
    KIT_ASSERT_ERROR(min_angle <= m_max_angle, "Min angle must be less than max angle: {0} < {1}", min_angle,
                     m_max_angle);
    m_min_angle = min_angle;
    awake();
}

float ball_joint2D::max_angle() const
{
    return m_max_angle;
}
void ball_joint2D::max_angle(const float max_angle)
{
    KIT_ASSERT_ERROR(m_min_angle <= max_angle, "Min angle must be less than max angle: {0} < {1}", m_min_angle,
                     max_angle);
    m_max_angle = max_angle;
    awake();
}

void ball_joint2D::update_constraint_data()
{
    vconstraint2D<0, 1>::update_constraint_data();
    m_relangle = state1().centroid.rotation - state2().centroid.rotation;
    m_relangle -= glm::round(m_relangle / glm::two_pi<float>()) * glm::two_pi<float>();
    m_legal_angle = m_relangle >= m_min_angle && m_relangle <= m_max_angle;
    m_c = constraint_position();
}

} // namespace ppx