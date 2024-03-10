#include "ppx/internal/pch.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/serialization/serialization.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
distance_joint2D::distance_joint2D(world2D &world, const specs &spc)
    : constraint2D(world, world.bodies.ptr(spc.bindex1), world.bodies.ptr(spc.bindex2), spc.ganchor1, spc.ganchor2),
      length(glm::distance(ganchor1(), ganchor2()))
{
}

distance_joint2D::const_ptr distance_joint2D::as_ptr() const
{
    return world.joints.manager<distance_joint2D>()->ptr(index);
}
distance_joint2D::ptr distance_joint2D::as_ptr()
{
    return world.joints.manager<distance_joint2D>()->ptr(index);
}

float distance_joint2D::constraint_value() const
{
    KIT_ASSERT_ERROR(length >= 0.f, "Length must be non-negative: {0}", length)
    return length - glm::distance(ganchor1(), ganchor2());
}
float distance_joint2D::constraint_velocity() const
{
    return glm::dot(m_dir, m_body2->ctr_proxy.velocity_at_centroid_offset(m_offset2) -
                               m_body1->ctr_proxy.velocity_at_centroid_offset(m_offset1));
}

float distance_joint2D::inverse_mass() const
{
    const float cross1 = kit::cross2D(m_offset1, m_dir);
    const float cross2 = kit::cross2D(m_offset2, m_dir);
    return m_body1->props().dynamic.inv_mass + m_body2->props().dynamic.inv_mass +
           m_body1->props().dynamic.inv_inertia * cross1 * cross1 +
           m_body2->props().dynamic.inv_inertia * cross2 * cross2;
}

glm::vec2 distance_joint2D::direction() const
{
    return m_offset2 - m_offset1 + m_body2->centroid() - m_body1->centroid();
}

void distance_joint2D::solve()
{
    solve_unclamped();
}
} // namespace ppx