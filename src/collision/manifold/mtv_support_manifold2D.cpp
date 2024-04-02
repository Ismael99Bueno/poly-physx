#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/mtv_support_manifold2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
manifold2D mtv_support_manifold2D::polygon_polygon_contacts(const collision2D &current,
                                                            const collision2D *previous) const
{
    const polygon &poly1 = current.collider1->shape<polygon>();
    const polygon &poly2 = current.collider2->shape<polygon>();
    manifold2D manifold = {geo::mtv_support_contact_point(poly1, poly2, current.mtv)};
    if (previous && glm::distance2(previous->manifold[0].point, manifold[0].point) > 0.04f)
        manifold.push_back(previous->manifold[0]);

    return manifold;
}
} // namespace ppx