#pragma once

#include "geo/shapes2D/circle.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class world2D;
class cc_manifold_algorithm2D
{
  public:
    virtual ~cc_manifold_algorithm2D() = default;
    virtual manifold2D circle_circle_contacts(const geo::circle &circ1, const geo::circle &circ2,
                                              const glm::vec2 &mtv) const = 0;
};

class cp_manifold_algorithm2D
{
  public:
    virtual ~cp_manifold_algorithm2D() = default;
    virtual manifold2D circle_polygon_contacts(const geo::circle &circ, const geo::polygon<8> &poly,
                                               const glm::vec2 &mtv) const = 0;
};

class pp_manifold_algorithm2D
{
  public:
    virtual ~pp_manifold_algorithm2D() = default;
    virtual manifold2D polygon_polygon_contacts(const geo::polygon<8> &poly1, const geo::polygon<8> &poly2,
                                                const glm::vec2 &mtv) const = 0;
};
} // namespace ppx