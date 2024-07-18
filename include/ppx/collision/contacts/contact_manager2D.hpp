#pragma once

#include "ppx/collision/contacts/contact2D.hpp"
#include "ppx/collision/contacts/collision_contacts2D.hpp"

namespace ppx
{
template <Contact2D Contact> class contact_manager2D : public collision_contacts2D
{
  public:
    using contact_map = std::unordered_map<contact_key, Contact *>;
    using collision_contacts2D::collision_contacts2D;

    virtual ~contact_manager2D()
    {
        destroy_all_contacts();
    }

    const std::vector<Contact *> &contacts() const
    {
        return m_contacts;
    }
    const contact_map &contacts_map() const
    {
        return m_unique_contacts;
    }

    std::vector<contact2D *> create_total_contacts_list() const override final
    {
        return std::vector<contact2D *>(m_contacts.begin(), m_contacts.end());
    }
    std::vector<contact2D *> create_active_contacts_list() const override final
    {
        std::vector<contact2D *> active_contacts;
        active_contacts.reserve(m_contacts.size());
        for (Contact *contact : m_contacts)
            if (contact->enabled()) [[likely]]
                active_contacts.push_back(contact);
        return active_contacts;
    }

    std::size_t total_contacts_count() const override final
    {
        return m_contacts.size();
    }

    void remove_any_contacts_with(const collider2D *collider) override final
    {
        for (auto it = m_contacts.begin(); it != m_contacts.end();)
        {
            Contact *contact = *it;
            if (contact->collider1() == collider || contact->collider2() == collider)
            {
                if (contact->enabled()) [[likely]]
                    contact->on_exit();
                m_unique_contacts.erase(contact->key());
                destroy_contact(contact);
                it = m_contacts.erase(it);
            }
            else
                ++it;
        }
    }

  protected:
    std::vector<Contact *> m_contacts;
    std::vector<Contact *> m_last_contacts;
    contact_map m_unique_contacts;

  private:
    void create_from_collision(const collision2D &collision) override final
    {
        for (std::size_t i = 0; i < collision.manifold.size(); i++)
        {
            const contact_key hash{collision.collider1, collision.collider2, collision.manifold[i].id.key};
            create_contact(hash, &collision, i);
        }
    }
    void update_from_collision(collision2D &collision) override final
    {
        for (std::size_t i = collision.manifold.size() - 1; i < collision.manifold.size(); i--)
        {
            const contact_key hash{collision.collider1, collision.collider2, collision.manifold[i].id.key};
            const auto old_contact = m_unique_contacts.find(hash);
            if (old_contact != m_unique_contacts.end())
            {
                update_contact(old_contact->second, &collision, i);
                collision.manifold.erase(collision.manifold.begin() + i);
            }
        }
    }
    void create_or_update_from_collision(const collision2D &collision) override final
    {
        for (std::size_t i = 0; i < collision.manifold.size(); i++)
        {
            const contact_key hash{collision.collider1, collision.collider2, collision.manifold[i].id.key};
            const auto old_contact = m_unique_contacts.find(hash);
            if (old_contact != m_unique_contacts.end())
                update_contact(old_contact->second, &collision, i);
            else
                create_contact(hash, &collision, i);
        }
    }
    void remove_expired_contacts() override final
    {
        std::swap(m_last_contacts, m_contacts);
        m_contacts.clear();
        for (Contact *contact : m_last_contacts)
        {
            if (contact->asleep())
            {
                m_contacts.push_back(contact);
                continue;
            }
            if (contact->expired())
            {
                m_unique_contacts.erase(contact->key());
                destroy_contact(contact);
                continue;
            }
            if (!contact->recently_updated())
            {
                contact->enabled(false);
                contact->on_exit();
            }
            contact->increment_age();
            m_contacts.push_back(contact);
        }
    }

    void create_contact(const contact_key &hash, const collision2D *collision, const std::size_t manifold_index)
    {
        Contact *contact = allocator<Contact>::create(world, collision, manifold_index);
        KIT_ASSERT_ERROR(!m_unique_contacts.contains(hash), "Contact already exists!")

        m_unique_contacts.emplace(hash, contact);
        m_contacts.push_back(contact);
        island2D::add(contact);
        contact->body1()->meta.contacts.push_back(contact);
        contact->body2()->meta.contacts.push_back(contact);
        contact->on_enter();
    }
    void update_contact(Contact *contact, const collision2D *collision, const std::size_t manifold_index)
    {
        KIT_ASSERT_WARN(!contact->recently_updated(),
                        "Contact was already updated. The corresponding collision is duplicated!")
        if (contact->recently_updated())
            return;

        const bool on_enter = !contact->enabled();
        contact->update(collision, manifold_index);
        if (on_enter)
            contact->on_enter();
    }
    void destroy_contact(Contact *contact)
    {
        contact->body1()->meta.remove_contact(contact);
        contact->body2()->meta.remove_contact(contact);
        island2D::remove(contact);
        allocator<Contact>::destroy(contact);
    }

    void destroy_all_contacts() override final
    {
        for (Contact *contact : m_contacts)
        {
            if (contact->enabled()) [[likely]]
                contact->on_exit();
            destroy_contact(contact);
        } // this is expensive, for every contact a call to remove_contact for
          // colliders and bodies. it shouldnt matter because this is almost never
          // called (only when user changes contact solver or disables collisions)
        m_contacts.clear();
        m_unique_contacts.clear();
    }
};
} // namespace ppx