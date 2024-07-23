#include "ppx/internal/pch.hpp"
#include "ppx/actuators/actuator.hpp"
#include "ppx/body/body.hpp"
#include "ppx/world.hpp"

namespace ppx
{
bool actuator2D::is_actuator() const
{
    return true;
}

void actuator2D::solve(std::vector<state2D> &states)
{
    const std::size_t index1 = m_body1->meta.index;
    const std::size_t index2 = m_body2->meta.index;

    state2D &state1 = states[index1];
    state2D &state2 = states[index2];

    compute_anchors_and_offsets(state1, state2);
    const glm::vec3 f = compute_force(state1, state2);
    m_force = glm::vec2(f);
    m_torque = f.z;

    const float t1 = kit::cross2D(m_offset1, m_force) + m_torque;
    const float t2 = kit::cross2D(m_offset2, m_force) + m_torque;

    state1.substep_force -= m_force;
    state1.substep_torque -= t1;

    state2.substep_force += m_force;
    state2.substep_torque += t2;
}

glm::vec2 actuator2D::reactive_force() const
{
    return m_force;
}
float actuator2D::reactive_torque() const
{
    return m_torque;
}

} // namespace ppx