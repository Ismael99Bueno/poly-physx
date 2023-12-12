#ifndef PPX_CONSTRAINT_DRIVEN_SOLVER2D_HPP
#define PPX_CONSTRAINT_DRIVEN_SOLVER2D_HPP

#include "ppx/collision/solvers/collision_solver2D.hpp"

namespace ppx
{
class constraint_driven_solver2D : public collision_solver2D
{
    void solve(const std::vector<collision2D> &collisions) const override;
};
} // namespace ppx

#endif