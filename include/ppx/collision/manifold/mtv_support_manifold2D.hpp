#pragma once

#include "ppx/collision/manifold/pp_manifold_algorithm2D.hpp"

namespace ppx
{
class mtv_support_manifold2D final : public pp_manifold_algorithm2D
{
  public:
    using pp_manifold_algorithm2D::pp_manifold_algorithm2D;

  private:
    manifold2D polygon_polygon_contacts(const collision2D &current) const override;
};
} // namespace ppx