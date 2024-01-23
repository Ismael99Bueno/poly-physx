#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/mtv_support_manifold2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
manifold2D mtv_support_manifold2D::circle_circle_contacts(const circle &circ1, const circle &circ2,
                                                          const glm::vec2 &mtv) const
{
    return {{geo::mtv_support_contact_point(circ1, circ2, mtv)}, 1};
}
manifold2D mtv_support_manifold2D::circle_polygon_contacts(const circle &circ, const polygon &poly,
                                                           const glm::vec2 &mtv) const
{
    return {{geo::mtv_support_contact_point(circ, poly, mtv)}, 1};
}
manifold2D mtv_support_manifold2D::polygon_polygon_contacts(const polygon &poly1, const polygon &poly2,
                                                            const glm::vec2 &mtv) const
{
    return {{geo::mtv_support_contact_point(poly1, poly2, mtv)}, 1};
}
} // namespace ppx