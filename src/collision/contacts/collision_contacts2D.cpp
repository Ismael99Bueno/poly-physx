#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/collision_contacts2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
collision_contacts2D::collision_contacts2D(world2D &world) : worldref2D(world)
{
}

void collision_contacts2D::inherit(collision_contacts2D &&contacts)
{
    params = contacts.params;
}

bool collision_contacts2D::enabled() const
{
    return m_enabled;
}

void collision_contacts2D::enabled(bool enable)
{
    m_enabled = enable;
    if (!enable)
        destroy_all_contacts();
}

bool collision_contacts2D::checksum() const
{
    std::unordered_set<const contact2D *> body_contacts;
    body_contacts.reserve(size());

    const auto contact_list = create_contacts_list();
    const std::unordered_set<const contact2D *> contacts(contact_list.begin(), contact_list.end());
    for (const body2D *body : world.bodies)
        for (const contact2D *contact : body->meta.contacts)
        {
            if (!contact->contains(body))
            {
                KIT_ERROR("Contact checksum failed: Contact does not contain body")
                return false;
            }
            if (!contacts.contains(contact))
            {
                KIT_ERROR("Contact checksum failed: Contact not found in contact list")
                return false;
            }
            body_contacts.insert(contact);
        }

    KIT_ASSERT_ERROR(contacts.size() == size(), "Found duplicate contacts in contact list")
    KIT_ASSERT_ERROR(body_contacts.size() == size(),
                     "Checksum failed: Contact count mismatch. Contacts count: {0} Body contacts count: {1}", size(),
                     body_contacts.size());
    return body_contacts.size() == size() && contacts.size() == size();
}

} // namespace ppx