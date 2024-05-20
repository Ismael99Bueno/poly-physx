#pragma once

#include "ppx/body/body2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/internal/worldref.hpp"
#include "kit/interface/toggleable.hpp"

namespace ppx
{
class collision_resolution2D : public worldref2D, public kit::toggleable, kit::non_copyable
{
  public:
    collision_resolution2D(world2D &world);
    virtual ~collision_resolution2D() = default;

    void solve(const collision_detection2D::collision_map &collisions);
    virtual void solve_collisions(const collision_detection2D::collision_map &collisions) = 0;

    void flag_new_step();

  private:
    virtual void on_attach()
    {
    }

    friend class collision_manager2D;
};
} // namespace ppx
