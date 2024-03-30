#pragma once

#include "ppx/common/shapes2D.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class pp_manifold_algorithm2D
{
  public:
    virtual ~pp_manifold_algorithm2D() = default;
    virtual manifold2D polygon_polygon_contacts(const polygon &poly1, const polygon &poly2,
                                                const glm::vec2 &mtv) const = 0;
};
} // namespace ppx