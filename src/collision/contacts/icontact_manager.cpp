#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/icontact_manager.hpp"
#include "ppx/world.hpp"

namespace ppx
{
void icontact_manager2D::inherit(icontact_manager2D &&contacts)
{
    params = contacts.params;
}

void icontact_manager2D::enabled(const bool enabled)
{
    m_enabled = enabled;
    if (!enabled)
        destroy_all_contacts();
}

bool icontact_manager2D::checksum(const body_manager2D &bm) const
{
    std::unordered_set<const contact2D *> body_contacts;
    const std::size_t contacts_count = total_contacts_count();
    body_contacts.reserve(contacts_count);

    const auto contact_list = create_total_contacts_list();
    const std::unordered_set<const contact2D *> contacts(contact_list.begin(), contact_list.end());
    for (const body2D *body : bm)
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

    KIT_ASSERT_ERROR(contacts.size() == contacts_count, "Found duplicate contacts in contact list")
    KIT_ASSERT_ERROR(body_contacts.size() == contacts_count,
                     "Checksum failed: Contact count mismatch. Contacts count: {0} Body contacts count: {1}",
                     contacts_count, body_contacts.size());
    return body_contacts.size() == contacts_count && contacts.size() == contacts_count;
}

} // namespace ppx