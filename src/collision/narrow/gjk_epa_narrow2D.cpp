#include "ppx/internal/pch.hpp"
#include "ppx/collision/narrow/gjk_epa_narrow2D.hpp"
#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
gjk_epa_narrow2D::gjk_epa_narrow2D(world2D &world, float epa_threshold)
    : narrow_phase2D(world), epa_threshold(epa_threshold)
{
}

const char *gjk_epa_narrow2D::name() const
{
    return "GJK-EPA";
}

narrow_phase2D::result gjk_epa_narrow2D::circle_polygon(const circle &circ, const polygon &poly) const
{
    result result = gjk_epa(circ, poly);
    if (!result)
        return result;
    result.manifold = {geo::radius_penetration_contact_point(circ, result.mtv)};
    return result;
}

narrow_phase2D::result gjk_epa_narrow2D::polygon_polygon(const polygon &poly1, const polygon &poly2) const
{
    result result = gjk_epa(poly1, poly2);
    if (!result)
        return result;
    result.manifold = geo::clipping_contacts(poly1, poly2, result.mtv);
    result.intersects = !result.manifold.empty();
    return result;
}

narrow_phase2D::result gjk_epa_narrow2D::gjk_epa(const shape2D &sh1, const shape2D &sh2) const
{
    result result;
    const geo::gjk_result2D gres = geo::gjk(sh1, sh2);
    if (!gres)
        return result;

    const geo::mtv_result2D mres = geo::epa(sh1, sh2, gres.simplex, epa_threshold);
    if (!mres)
        return result;
    result.intersects = true;
    result.mtv = mres.mtv;
    return result;
}

} // namespace ppx