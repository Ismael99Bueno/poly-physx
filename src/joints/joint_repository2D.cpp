#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_repository2D.hpp"

namespace ppx
{
joint_repository2D::joint_repository2D(world2D &world) : non_constraint_based(world), constraint_based(world)
{
}
} // namespace ppx