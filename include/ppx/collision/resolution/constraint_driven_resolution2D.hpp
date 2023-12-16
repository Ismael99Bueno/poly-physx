#ifndef PPX_CONSTRAINT_driven_resolution2D_HPP
#define PPX_CONSTRAINT_driven_resolution2D_HPP

#include "ppx/collision/resolution/collision_resolution2D.hpp"

namespace ppx
{
class constraint_driven_resolution2D : public collision_resolution2D
{
    void solve(const std::vector<collision2D> &collisions) const override;
};
} // namespace ppx

#endif