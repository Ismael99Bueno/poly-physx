#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/collision_contacts2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
collision_contacts2D::collision_contacts2D(world2D &world) : worldref2D(world)
{
}

bool collision_contacts2D::contacts_checksum() const
{
    std::size_t count = 0;
    for (const body2D *body : world.bodies)
        count += body->meta.contacts.size();
    KIT_ASSERT_ERROR(count == size(),
                     "Checksum failed: Contact count mismatch. Contacts count: {0} Body contacts count: {1}", size(),
                     count);
    return count == size();
}

void collision_contacts2D::global_on_contact_enter(world2D &world, contact2D *contact)
{
    world.collisions.events.on_contact_enter(contact);
}

void collision_contacts2D::global_on_contact_pre_solve(world2D &world, contact2D *contact)
{
    world.collisions.events.on_contact_pre_solve(contact);
}

void collision_contacts2D::global_on_contact_post_solve(world2D &world, contact2D *contact)
{
    world.collisions.events.on_contact_post_solve(contact);
}
void collision_contacts2D::global_on_contact_exit(world2D &world, contact2D &contact)
{
    world.collisions.events.on_contact_exit(contact);
}

} // namespace ppx