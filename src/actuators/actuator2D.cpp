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
    const glm::vec4 f = compute_force();
    m_force = glm::vec2(f);
    m_torque = f.z + f.w;

    m_body1->apply_simulation_force(-m_force);
    m_body2->apply_simulation_force(m_force);

    m_body1->apply_simulation_torque(f.z);
    m_body2->apply_simulation_torque(f.w);
}

} // namespace ppx