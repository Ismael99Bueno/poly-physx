#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
constraint_driven_resolution2D::constraint_driven_resolution2D(const float restitution, const float friction,
                                                               const float slop)
    : restitution(restitution), friction(friction), slop(slop)
{
}
void constraint_driven_resolution2D::solve(const std::vector<collision2D> &collisions) const
{
    KIT_ASSERT_ERROR(friction >= 0.f, "Friction must be non-negative: %f", friction)
    KIT_ASSERT_ERROR(restitution >= 0.f, "Restitution must be non-negative: %f", restitution)
    KIT_ASSERT_ERROR(slop >= 0.f, "Slop must be non-negative: %f", slop)
    world->constraints.delegate_collisions(&collisions);
}
} // namespace ppx