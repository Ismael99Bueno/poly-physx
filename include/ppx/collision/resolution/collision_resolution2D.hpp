#pragma once

#include "ppx/entities/body2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/internal/worldref.hpp"

namespace ppx
{
class collision_resolution2D : public worldref2D
{
  public:
    collision_resolution2D(world2D &world);
    virtual ~collision_resolution2D() = default;
    virtual void solve(const std::vector<collision2D> &collisions) = 0;

  private:
    virtual void on_attach()
    {
    }

    friend class collision_manager2D;
};
} // namespace ppx
