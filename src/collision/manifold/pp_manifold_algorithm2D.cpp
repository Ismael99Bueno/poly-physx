#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/pp_manifold_algorithm2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
pp_manifold_algorithm2D::pp_manifold_algorithm2D(world2D &world) : worldref2D(world)
{
}

bool pp_manifold_algorithm2D::must_recompute(const collision2D &current, const collision2D *previous) const
{
    if (!previous)
        return true;
    const float ts = world.rk_substep_timestep();
    if (current.angular_velocity1 * ts > 0.01f || current.angular_velocity2 * ts > 0.01f ||
        previous->angular_velocity1 * ts > 0.01f || previous->angular_velocity2 * ts > 0.01f)
        return true;

    const glm::vec2 disp1 = current.velocity2 - current.velocity1;
    const glm::vec2 disp2 = previous->velocity2 - previous->velocity1;
    if (glm::length2(disp1) * ts > 0.1f || glm::length2(disp2) * ts > 0.1f)
        return true;
    return false;
}

manifold2D pp_manifold_algorithm2D::recycle_previous_manifold(const collision2D &current,
                                                              const collision2D *previous) const
{
    manifold2D result = previous->manifold;
    const body2D *body1 = current.collider1->body();
    const body2D *body2 = current.collider2->body();
    for (geo::contact_point2D &contact : result)
    {
        const glm::vec2 offset1 = contact.point - body1->centroid();
        const glm::vec2 offset2 = contact.point - body2->centroid();

        const glm::vec2 rot_offset1 = glm::vec2(-offset1.y, offset1.x);
        const glm::vec2 rot_offset2 = glm::vec2(-offset2.y, offset2.x);

        const glm::vec2 cvel1 = current.velocity1 + current.angular_velocity1 * rot_offset1;
        const glm::vec2 cvel2 = current.velocity2 + current.angular_velocity2 * rot_offset2;

        const glm::vec2 pvel1 = previous->velocity1 + previous->angular_velocity1 * rot_offset1;
        const glm::vec2 pvel2 = previous->velocity2 + previous->angular_velocity2 * rot_offset2;

        const glm::vec2 vel1 = 0.5f * (cvel1 + pvel1);
        const glm::vec2 vel2 = 0.5f * (cvel2 + pvel2);
        const glm::vec2 relvel = vel2 - vel1;

        contact.point -= relvel * world.rk_substep_timestep();
    }
    return result;
}
} // namespace ppx