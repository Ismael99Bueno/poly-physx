#pragma once

#include "ppx/collision/manifold/pp_manifold_algorithm2D.hpp"

namespace ppx
{
class clipping_algorithm_manifold2D final : public pp_manifold_algorithm2D
{
  public:
    clipping_algorithm_manifold2D(bool allow_intersections = false);
    bool allow_intersections = false;

  private:
    manifold2D polygon_polygon_contacts(const polygon &poly1, const polygon &poly2,
                                        const glm::vec2 &mtv) const override;
};
} // namespace ppx