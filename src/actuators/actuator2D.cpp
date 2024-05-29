#include "ppx/internal/pch.hpp"
#include "ppx/actuators/actuator2D.hpp"
#include "ppx/body/body2D.hpp"

namespace ppx
{
bool actuator2D::is_constraint() const
{
    return false;
}
bool actuator2D::is_actuator() const
{
    return true;
}

void actuator2D::solve()
{
    compute_anchors_and_offsets();
    const glm::vec3 f = compute_force();
    m_force = glm::vec2(f);
    m_torque = f.z;

    m_body1->apply_simulation_force(-m_force);
    m_body2->apply_simulation_force(m_force);

    const float t1 = kit::cross2D(m_force, m_offset1);
    const float t2 = kit::cross2D(m_offset2, m_force);

    m_body1->apply_simulation_torque(t1);
    m_body2->apply_simulation_torque(t2);
}

} // namespace ppx