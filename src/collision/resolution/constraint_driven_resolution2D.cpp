#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
constraint_driven_resolution2D::constraint_driven_resolution2D(world2D &world, const float slop)
    : collision_resolution2D(world), slop(slop)
{
}
void constraint_driven_resolution2D::solve(const std::vector<collision2D> &collisions) const
{
    KIT_ASSERT_ERROR(slop >= 0.f, "Slop must be non-negative: {0}", slop)
    world.constraints.delegate_collisions(&collisions);
}
} // namespace ppx