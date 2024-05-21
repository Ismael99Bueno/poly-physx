#include "ppx/internal/pch.hpp"
#include "ppx/joints/ball_joint2D.hpp"
#include "ppx/body/body2D.hpp"

namespace ppx
{
ball_joint2D::ball_joint2D(world2D &world, const specs &spc) : pvconstraint2D<0, 1>{world, spc}, props{spc.props}
{
    if (spc.deduce_angle)
    {
        m_relangle = m_body2->meta.ctr_state.centroid.rotation() - m_body1->meta.ctr_state.centroid.rotation();
        props.min_angle = m_relangle;
        props.max_angle = m_relangle;
    }
}

float ball_joint2D::constraint_position() const
{
    KIT_ASSERT_ERROR(props.min_angle <= props.max_angle, "Min angle must be less than max angle: {0} < {1}",
                     props.min_angle, props.max_angle);

    if (m_relangle < props.min_angle)
        return m_relangle - props.min_angle;
    if (m_relangle > props.max_angle)
        return m_relangle - props.max_angle;
    return 0.f;
}

float ball_joint2D::constraint_velocity() const
{
    return m_body2->meta.ctr_state.angular_velocity - m_body1->meta.ctr_state.angular_velocity;
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
    if (kit::approximately(props.min_angle, props.max_angle))
        pvconstraint2D<0, 1>::solve_velocities();
    else if (m_relangle < props.min_angle)
        solve_velocities_clamped(0.f, FLT_MAX);
    else
        solve_velocities_clamped(-FLT_MAX, 0.f);
}

void ball_joint2D::update_constraint_data()
{
    vconstraint2D<0, 1>::update_constraint_data();
    m_relangle = m_body2->meta.ctr_state.centroid.rotation() - m_body1->meta.ctr_state.centroid.rotation();
    m_relangle -= glm::round(m_relangle / glm::two_pi<float>()) * glm::two_pi<float>();
    m_legal_angle = m_relangle >= props.min_angle && m_relangle <= props.max_angle;
    m_c = constraint_position();
}

} // namespace ppx