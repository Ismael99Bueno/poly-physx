#pragma once

#include "ppx/collision/manifold/manifold_generators2D.hpp"

namespace ppx
{
class radius_distance_manifold2D : public circle_circle_manifold2D
{
    manifold2D circle_circle_contacts(const geo::circle &circ1, const geo::circle &circ2,
                                      const glm::vec2 &mtv) const override;
};
} // namespace ppx