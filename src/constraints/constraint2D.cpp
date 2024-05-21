#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint2D.hpp"

namespace ppx
{
bool constraint2D::solve_positions()
{
    return true;
}

bool constraint2D::is_constraint() const
{
    return true;
}

void constraint2D::solve()
{
    solve_velocities();
    solve_positions();
}

} // namespace ppx