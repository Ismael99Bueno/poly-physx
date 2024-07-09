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

    std::vector<contact2D *> create_total_contacts_list() const override final
    {
        std::vector<contact2D *> list;
        list.reserve(m_contacts.size());
        for (const auto &pair : m_contacts)
            list.push_back(pair.second);
        return list;
    }
    std::vector<contact2D *> create_active_contacts_list() const override final
    {
        return std::vector<contact2D *>(m_active_contacts.begin(), m_active_contacts.end());
    }

    std::size_t total_contacts_count() const override final
    {
        return m_contacts.size();
    }
    std::size_t active_contacts_count() const override final
    {
        return m_active_contacts.size();
    }

    void remove_any_contacts_with(const collider2D *collider) override final
    {
        for (auto it = m_contacts.begin(); it != m_contacts.end();)
        {
            Contact *contact = it->second;
            if (contact->collider1() == collider || contact->collider2() == collider)
            {
                if (contact->enabled()) [[likely]]
                    contact->on_exit();
                destroy_contact(contact);
                it = m_contacts.erase(it);
            }
            else
                ++it;
        }
    }

  protected:
    contact_map m_contacts;
    std::vector<Contact *> m_active_contacts;

    void create_contacts_from_collisions(const std::vector<collision2D> &collisions) override final
    {
        KIT_PERF_FUNCTION()
        m_active_contacts.clear();

        for (const collision2D &collision : collisions)
        {
            for (std::size_t i = 0; i < collision.manifold.size(); i++)
            {
                const contact_key hash{collision.collider1, collision.collider2, collision.manifold[i].id.key};
                const auto old_contact = m_contacts.find(hash);
                if (old_contact != m_contacts.end())
                    update_contact(old_contact->second, &collision, i);
                else
                    create_contact(hash, &collision, i);
            }
        }

        if (rk_substep_index() == 0)
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
                    contact->enabled(false);
                    contact->on_exit();
                }
                contact->increment_age();
                ++it;
            }
    }

    void create_contact(const contact_key &hash, const collision2D *collision, const std::size_t manifold_index)
    {
        KIT_PERF_FUNCTION()
        Contact *contact = allocator<Contact>::create(world, collision, manifold_index);
        m_contacts.emplace(hash, contact);
        island2D::add(contact);
        contact->body1()->meta.contacts.push_back(contact);
        contact->body2()->meta.contacts.push_back(contact);
        contact->on_enter();
        if (contact->enabled()) [[likely]]
            m_active_contacts.push_back(contact);
    }
    void update_contact(Contact *contact, const collision2D *collision, const std::size_t manifold_index)
    {
        KIT_PERF_FUNCTION()

        KIT_ASSERT_WARN(!contact->recently_updated(),
                        "Contact was already updated. The corresponding collision is duplicated!")
        if (contact->recently_updated())
            return;

        const bool on_enter = !contact->enabled();
        contact->update(collision, manifold_index);
        if (on_enter)
            contact->on_enter();
        if (contact->enabled()) [[likely]]
            m_active_contacts.push_back(contact);
    }
    void destroy_contact(Contact *contact)
    {
        KIT_PERF_FUNCTION()
        contact->body1()->meta.remove_contact(contact);
        contact->body2()->meta.remove_contact(contact);
        island2D::remove(contact);
        allocator<Contact>::destroy(contact);
    }

  private:
    void destroy_all_contacts() override final
    {
        for (auto &pair : m_contacts)
        {
            if (pair.second->enabled()) [[likely]]
                pair.second->on_exit();
            destroy_contact(pair.second);
        } // this is expensive, for every contact a call to remove_contact for
          // colliders and bodies. it shouldnt matter because this is almost never
          // called (only when user changes contact solver or disables collisions)
        m_contacts.clear();
        m_active_contacts.clear();
    }
};
} // namespace ppx