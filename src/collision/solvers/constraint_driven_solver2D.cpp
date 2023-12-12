#include "ppx/internal/pch.hpp"
#include "ppx/collision/solvers/constraint_driven_solver2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void constraint_driven_solver2D::solve(const std::vector<collision2D> &collisions) const
{
    world->constraints.delegate_collisions(&collisions);
}
} // namespace ppx