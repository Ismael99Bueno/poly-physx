#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/mtv_support_manifold2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
manifold2D mtv_support_manifold2D::polygon_polygon_contacts(const polygon &poly1, const polygon &poly2,
                                                            const glm::vec2 &mtv) const
{
    return {{geo::mtv_support_contact_point(poly1, poly2, mtv), {}}};
}
} // namespace ppx