#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_repository2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
joint_repository2D::joint_repository2D(world2D &world)
    : manager2D(world), actuators(world, m_elements, events), constraints(world, m_elements, events)
{
}

bool joint_repository2D::remove_manager(const std::size_t index)
{
    if (actuators.remove(index))
        return true;
    return constraints.remove(index);
}

bool joint_repository2D::remove(joint2D *joint)
{
    if (actuators.remove(joint))
        return true;
    return constraints.remove(joint);
}
bool joint_repository2D::remove(const std::size_t index)
{
    return remove(m_elements[index]);
}

} // namespace ppx