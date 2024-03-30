#pragma once

#include "ppx/collision/manifold/pp_manifold_algorithm2D.hpp"

namespace ppx
{
class mtv_support_manifold2D final : public pp_manifold_algorithm2D
{
    manifold2D polygon_polygon_contacts(const polygon &poly1, const polygon &poly2,
                                        const glm::vec2 &mtv) const override;
};
} // namespace ppx