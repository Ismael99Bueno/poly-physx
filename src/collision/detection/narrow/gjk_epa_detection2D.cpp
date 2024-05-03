#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/narrow/gjk_epa_detection2D.hpp"
#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
gjk_epa_detection2D::gjk_epa_detection2D(float epa_threshold) : epa_threshold(epa_threshold)
{
}

narrow_result gjk_epa_detection2D::circle_polygon(const circle &circ, const polygon &poly) const
{
    return gjk_epa(circ, poly);
}

narrow_result gjk_epa_detection2D::polygon_polygon(const polygon &poly1, const polygon &poly2) const
{
    return gjk_epa(poly1, poly2);
}

narrow_result gjk_epa_detection2D::gjk_epa(const shape2D &sh1, const shape2D &sh2) const
{
    narrow_result result;
    const geo::gjk_result2D gres = geo::gjk(sh1, sh2);
    if (!gres.intersect)
        return result;

    const geo::mtv_result2D mres = geo::epa(sh1, sh2, gres.simplex, epa_threshold);
    if (!mres.valid)
        return result;
    result.valid = true;
    result.mtv = mres.mtv;
    return result;
}

} // namespace ppx