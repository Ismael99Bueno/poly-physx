#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/radius_distance_manifold2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
manifold2D radius_distance_manifold2D::circle_circle_contacts(const circle &circ1, const circle &circ2,
                                                              const glm::vec2 &mtv) const
{
    return {{geo::radius_distance_contact_point(circ1, circ2)}, 1};
}
} // namespace ppx