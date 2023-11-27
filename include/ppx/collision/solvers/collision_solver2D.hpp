#ifndef PPX_COLLISION_SOLVER2D_HPP
#define PPX_COLLISION_SOLVER2D_HPP

#include "ppx/body2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"

namespace ppx
{
class collision_solver2D
{
  public:
    virtual ~collision_solver2D() = default;
    virtual void solve(const std::vector<collision2D> &collisions) const = 0;

    static inline float restitution_coeff = 0.4f;
    static inline float friction_coeff = 0.6f;

    world2D *world = nullptr;

  protected:
  private:
    virtual void on_attach()
    {
    }

    friend class collision_manager2D;
};
} // namespace ppx

#endif