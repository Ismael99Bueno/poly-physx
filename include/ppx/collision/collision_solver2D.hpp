#ifndef PPX_COLLISION_SOLVER2D_HPP
#define PPX_COLLISION_SOLVER2D_HPP

#include "ppx/body2D.hpp"
#include "ppx/collision/collision_detection2D.hpp"

namespace ppx
{
class collision_solver2D
{
  public:
    virtual ~collision_solver2D() = default;
    virtual void solve(const std::vector<collision2D> &collisions, std::vector<float> &state_derivative) const = 0;
};
} // namespace ppx

#endif