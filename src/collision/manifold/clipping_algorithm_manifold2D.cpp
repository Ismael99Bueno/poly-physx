#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/clipping_algorithm_manifold2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
manifold2D clipping_algorithm_manifold2D::polygon_polygon_contacts(const polygon &poly1, const polygon &poly2,
                                                                   const glm::vec2 &mtv) const
{
    return geo::clipping_contacts<manifold2D::CAPACITY>(poly1, poly2, mtv, false);
}
} // namespace ppx