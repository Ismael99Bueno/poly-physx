#include "ppx/internal/pch.hpp"
#include "ppx/collision/narrow/sat_narrow2D.hpp"
#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
narrow_result2D sat_narrow2D::circle_polygon(const circle &circ, const polygon &poly) const
{
    narrow_result2D result;
    geo::sat_result2D sat_result = geo::sat(circ, poly);
    if (!sat_result)
        return result;
    result.intersects = true;
    result.mtv = sat_result.mtv;
    result.manifold = {geo::radius_penetration_contact_point(circ, result.mtv)};
    return result;
}

narrow_result2D sat_narrow2D::polygon_polygon(const polygon &poly1, const polygon &poly2) const
{
    narrow_result2D result;
    geo::sat_result2D sat_result = geo::sat(poly1, poly2);
    if (!sat_result)
        return result;
    result.manifold = geo::clipping_contacts(poly1, poly2, sat_result.mtv);
    if (result.manifold.empty())
        return result;
    result.intersects = true;
    result.mtv = sat_result.mtv;
    return result;
}

} // namespace ppx