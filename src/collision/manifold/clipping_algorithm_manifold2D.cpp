#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/clipping_algorithm_manifold2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
manifold2D clipping_algorithm_manifold2D::polygon_polygon_contacts(const geo::polygon &poly1, const geo::polygon &poly2,
                                                                   const glm::vec2 &mtv) const
{
    const geo::clip_info<manifold2D::CAPACITY> result =
        geo::clipping_contacts<manifold2D::CAPACITY>(poly1, poly2, mtv, false);
    return {result.contacts, result.size};
}
} // namespace ppx