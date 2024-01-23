#pragma once

#include "ppx/entities/shapes2D.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class world2D;
class cc_manifold_algorithm2D
{
  public:
    virtual ~cc_manifold_algorithm2D() = default;
    virtual manifold2D circle_circle_contacts(const circle &circ1, const circle &circ2, const glm::vec2 &mtv) const = 0;
};

class cp_manifold_algorithm2D
{
  public:
    virtual ~cp_manifold_algorithm2D() = default;
    virtual manifold2D circle_polygon_contacts(const circle &circ, const polygon &poly, const glm::vec2 &mtv) const = 0;
};

class pp_manifold_algorithm2D
{
  public:
    virtual ~pp_manifold_algorithm2D() = default;
    virtual manifold2D polygon_polygon_contacts(const polygon &poly1, const polygon &poly2,
                                                const glm::vec2 &mtv) const = 0;
};
} // namespace ppx