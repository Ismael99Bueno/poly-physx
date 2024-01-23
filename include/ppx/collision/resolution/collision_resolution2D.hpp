#pragma once

#include "ppx/entities/body2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"

namespace ppx
{
class collision_resolution2D
{
  public:
    virtual ~collision_resolution2D() = default;
    virtual void solve(const std::vector<collision2D> &collisions) const = 0;

    world2D *world = nullptr;

  private:
    virtual void on_attach()
    {
    }

    friend class collision_manager2D;
};
} // namespace ppx
