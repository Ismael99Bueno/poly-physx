#ifndef PPX_CONSTRAINT_SOLVER2D_HPP
#define PPX_CONSTRAINT_SOLVER2D_HPP

#include "ppx/collision/solvers/collision_solver2D.hpp"

namespace ppx
{
class constraint_manager2D;
class constraint_solver2D : public collision_solver2D
{
    void solve(const std::vector<collision2D> &collisions) const override;

    constraint_manager2D *m_manager = nullptr;
};
} // namespace ppx

#endif