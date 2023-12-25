#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"

namespace ppx
{
class constraint_driven_resolution2D : public collision_resolution2D
{
  public:
    constraint_driven_resolution2D(float restitution = 0.4f, float friction = 0.6f);

    float restitution;
    float friction;

  private:
    void solve(const std::vector<collision2D> &collisions) const override;
};
} // namespace ppx
