#pragma once

#include "ppx/collision/manifold/manifold_generators2D.hpp"

namespace ppx
{
class mtv_support_manifold2D : public circle_circle_manifold2D,
                               public polygon_circle_manifold2D,
                               public polygon_polygon_manifold2D
{
    manifold2D circle_circle_contacts(const geo::circle &circ1, const geo::circle &circ2,
                                      const glm::vec2 &mtv) const override;
    manifold2D polygon_circle_contacts(const geo::polygon &poly, const geo::circle &circ,
                                       const glm::vec2 &mtv) const override;
    manifold2D polygon_polygon_contacts(const geo::polygon &poly1, const geo::polygon &poly2,
                                        const glm::vec2 &mtv) const override;
};
} // namespace ppx