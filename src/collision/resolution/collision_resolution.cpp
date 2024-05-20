#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/collision_resolution2D.hpp"

namespace ppx
{
collision_resolution2D::collision_resolution2D(world2D &world) : worldref2D(world)
{
}

void collision_resolution2D::resolve_into_contacts(const collision_detection2D::collision_map &collisions)
{
    for (const auto &collision : collisions)
    {
        if (!collision.second.collided || !collision.second.enabled)
            continue;
        collision.second.collider1->events.on_collision_pre_solve(collision.second);
        collision.second.collider2->events.on_collision_pre_solve(collision.second);
    }
    resolve(collisions);
    for (const auto &collision : collisions)
    {
        if (!collision.second.collided || !collision.second.enabled)
            continue;
        collision.second.collider1->events.on_collision_post_solve(collision.second);
        collision.second.collider2->events.on_collision_post_solve(collision.second);
    }
}
} // namespace ppx