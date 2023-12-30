#pragma once

#include "ppx/collision/manifold/manifold_algorithms2D.hpp"

namespace ppx
{
class radius_distance_manifold2D final : public cc_manifold_algorithm2D
{
    manifold2D circle_circle_contacts(const geo::circle &circ1, const geo::circle &circ2,
                                      const glm::vec2 &mtv) const override;
};
} // namespace ppx