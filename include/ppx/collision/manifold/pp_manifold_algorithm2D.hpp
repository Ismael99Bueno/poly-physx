#pragma once

#include "ppx/common/alias.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class pp_manifold_algorithm2D : public worldref2D
{
  public:
    pp_manifold_algorithm2D(world2D &world);

    virtual ~pp_manifold_algorithm2D() = default;
    virtual manifold2D polygon_polygon_contacts(const collision2D &current) const = 0;

  protected:
    // bool must_recompute(const collision2D &current, const collision2D *previous) const;
    // manifold2D recycle_previous_manifold(const collision2D &current, const collision2D *previous) const;
};
} // namespace ppx