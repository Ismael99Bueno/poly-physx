#pragma once

#include "ppx/collision/manifold/manifold_algorithms2D.hpp"

namespace ppx
{
class clipping_algorithm_manifold2D final : public pp_manifold_algorithm2D
{
    manifold2D polygon_polygon_contacts(const geo::polygon<8> &poly1, const geo::polygon<8> &poly2,
                                        const glm::vec2 &mtv) const override;
};
} // namespace ppx