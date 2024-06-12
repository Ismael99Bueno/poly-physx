#pragma once

#include "ppx/collision/contacts/contact2D.hpp"
#include "ppx/collision/contacts/collision_contacts2D.hpp"

namespace ppx
{
template <Contact2D Contact> class contact_manager2D : public collision_contacts2D
{
  public:
    virtual ~contact_manager2D()
    {
        destroy_all_contacts();
    }

    using contact_map = std::unordered_map<contact_key, Contact *>;

    using collision_contacts2D::collision_contacts2D;

    const contact_map &contacts() const
    {
        return m_contacts;
    }

    std::vector<contact2D *> create_contacts_list() const override
    {
        std::vector<contact2D *> list;
        list.reserve(m_contacts.size());
        for (const auto &pair : m_contacts)
            list.push_back(pair.second);
        return list;
    }

    std::size_t size() const override
    {
        return m_contacts.size();
    }

    void remove_any_contacts_with(const collider2D *collider) override
    {
        for (auto it = m_contacts.begin(); it != m_contacts.end();)
        {
            Contact *contact = it->second;
            if (contact->collider1() == collider || contact->collider2() == collider)
            {
                destroy_contact(contact);
                if (contact->enabled)
                    contact->on_exit();
                it = m_contacts.erase(it);
            }
            else
                ++it;
        }
    }

  protected:
    contact_map m_contacts;

    void create_contacts_from_collisions(const collision_detection2D::collision_map &collisions) override
    {
        KIT_PERF_FUNCTION()
        for (const auto &pair : collisions)
        {
            const collision2D &collision = pair.second;
            if (collision.asleep)
                continue;
            for (std::size_t i = 0; i < collision.manifold.size(); i++)
            {
                const contact_key hash{collision.collider1, collision.collider2, collision.manifold[i].id.key};
                const auto old_contact = m_contacts.find(hash);
                if (old_contact != m_contacts.end())
                {
                    if (!old_contact->second->enabled)
                        old_contact->second->on_enter();
                    old_contact->second->update(&collision, i);
                }
                else
                    create_contact(hash, &collision, i);
            }
        }
        for (auto it = m_contacts.begin(); it != m_contacts.end();)
        {
            Contact *contact = it->second;
            if (contact->asleep())
            {
                ++it;
                continue;
            }
            if (contact->expired())
            {
                destroy_contact(contact);
                it = m_contacts.erase(it);
                continue;
            }
            if (!contact->recently_updated())
            {
                contact->enabled = false;
                contact->on_exit();
            }
            contact->increment_lifetime();
            ++it;
        }
    }

    void create_contact(const contact_key &hash, const collision2D *collision, std::size_t manifold_index)
    {
        Contact *contact = allocator<Contact>::create(world, collision, manifold_index);
        m_contacts.emplace(hash, contact);
        island2D::add(contact);
        contact->body1()->meta.contacts.push_back(contact);
        contact->body2()->meta.contacts.push_back(contact);
        contact->on_enter();
    }
    void destroy_contact(Contact *contact)
    {
        contact->body1()->meta.remove_contact(contact);
        contact->body2()->meta.remove_contact(contact);
        island2D::remove(contact);
        allocator<Contact>::destroy(contact);
    }

  private:
    void destroy_all_contacts() override
    {
        for (auto &pair : m_contacts)
            destroy_contact(pair.second); // this is expensive, for every contact a call to remove_contact for
                                          // colliders and bodies. it shouldnt matter because this is almost never
                                          // called (only when user changes contact solver or disables collisions)
        m_contacts.clear();
    }
};
} // namespace ppx