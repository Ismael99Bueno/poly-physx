#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/mtv_support_manifold2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
manifold2D mtv_support_manifold2D::circle_circle_contacts(const geo::circle &circ1, const geo::circle &circ2,
                                                          const glm::vec2 &mtv) const
{
    return {{geo::mtv_support_contact_point(circ1, circ2, mtv)}, 1};
}
manifold2D mtv_support_manifold2D::polygon_circle_contacts(const geo::polygon &poly, const geo::circle &circ,
                                                           const glm::vec2 &mtv) const
{
    return {{geo::mtv_support_contact_point(poly, circ, mtv)}, 1};
}
manifold2D mtv_support_manifold2D::polygon_polygon_contacts(const geo::polygon &poly1, const geo::polygon &poly2,
                                                            const glm::vec2 &mtv) const
{
    return {{geo::mtv_support_contact_point(poly1, poly2, mtv)}, 1};
}
} // namespace ppx