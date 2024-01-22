#pragma once

#include "ppx/collision/manifold/manifold_algorithms2D.hpp"

namespace ppx
{
class mtv_support_manifold2D final : public cc_manifold_algorithm2D,
                                     public cp_manifold_algorithm2D,
                                     public pp_manifold_algorithm2D
{
    manifold2D circle_circle_contacts(const geo::circle &circ1, const geo::circle &circ2,
                                      const glm::vec2 &mtv) const override;
    manifold2D circle_polygon_contacts(const geo::circle &circ, const geo::polygon<> &poly,
                                       const glm::vec2 &mtv) const override;
    manifold2D polygon_polygon_contacts(const geo::polygon<> &poly1, const geo::polygon<> &poly2,
                                        const glm::vec2 &mtv) const override;
};
} // namespace ppx