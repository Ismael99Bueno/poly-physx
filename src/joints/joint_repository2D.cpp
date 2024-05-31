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

bool joint_repository2D::checksum() const
{
    std::unordered_set<const joint2D *> body_joints;
    std::unordered_set<const joint2D *> manager_joints;
    body_joints.reserve(size());
    manager_joints.reserve(size());

    const std::unordered_set<const joint2D *> joints(m_elements.begin(), m_elements.end());
    for (const body2D *body : world.bodies)
        for (const joint2D *joint : body->meta.joints)
        {
            if (!joint->contains(body))
            {
                KIT_ERROR("Joint checksum failed: Joint does not contain body")
                return false;
            }
            if (!joints.contains(joint))
            {
                KIT_ERROR("Joint checksum failed: Joint not found in joint list")
                return false;
            }
            body_joints.insert(joint);
        }

    KIT_ASSERT_ERROR(joints.size() == size(), "Found duplicate joints in joint list")
    KIT_ASSERT_ERROR(body_joints.size() == size(),
                     "Checksum failed: Contact count mismatch. Contacts count: {0} Body joints count: {1}", size(),
                     body_joints.size());
    return body_joints.size() == size() && joints.size() == size();
}

} // namespace ppx