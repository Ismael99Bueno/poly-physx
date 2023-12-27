#pragma once

#include "geo/shapes2D/circle.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class world2D;
class circle_circle_manifold2D
{
  public:
    virtual manifold2D circle_circle_contacts(const geo::circle &circ1, const geo::circle &circ2,
                                              const glm::vec2 &mtv) const = 0;
};

class polygon_circle_manifold2D
{
  public:
    virtual manifold2D polygon_circle_contacts(const geo::polygon &poly, const geo::circle &circ,
                                               const glm::vec2 &mtv) const = 0;
};

class polygon_polygon_manifold2D
{
  public:
    virtual manifold2D polygon_polygon_contacts(const geo::polygon &poly1, const geo::polygon &poly2,
                                                const glm::vec2 &mtv) const = 0;
};
} // namespace ppx