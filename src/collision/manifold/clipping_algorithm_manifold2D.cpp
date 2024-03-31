#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/clipping_algorithm_manifold2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
manifold2D clipping_algorithm_manifold2D::polygon_polygon_contacts(const collision2D &current,
                                                                   const collision2D *previous) const
{
    const polygon &poly1 = current.collider1->shape<polygon>();
    const polygon &poly2 = current.collider2->shape<polygon>();
    return geo::clipping_contacts(poly1, poly2, current.mtv);
}
} // namespace ppx