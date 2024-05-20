#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
collision_resolution2D::collision_resolution2D(world2D &world) : worldref2D(world)
{
}

void collision_resolution2D::global_on_contact_enter(world2D &world, contact2D *contact)
{
    world.collisions.events.on_contact_enter(contact);
}

void collision_resolution2D::global_on_contact_pre_solve(world2D &world, contact2D *contact)
{
    world.collisions.events.on_contact_pre_solve(contact);
}

void collision_resolution2D::global_on_contact_post_solve(world2D &world, contact2D *contact)
{
    world.collisions.events.on_contact_post_solve(contact);
}
void collision_resolution2D::global_on_contact_exit(world2D &world, contact2D &contact)
{
    world.collisions.events.on_contact_exit(contact);
}

} // namespace ppx