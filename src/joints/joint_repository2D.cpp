#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_repository2D.hpp"

namespace ppx
{
joint_repository2D::joint_repository2D(world2D &world) : non_constraint_based(world), constraint_based(world)
{
}

bool joint_repository2D::remove(const joint2D *joint)
{
    if (non_constraint_based.remove(joint))
        return true;
    return constraint_based.remove(joint);
}
} // namespace ppx