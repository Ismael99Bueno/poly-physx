#pragma once

#include "ppx/collision/contacts/contact.hpp"
#include "ppx/collision/contacts/icontact_manager.hpp"
#include "ppx/manager.hpp"

namespace ppx
{
template <Contact2D Contact> class contact_manager2D : public manager2D<Contact>, virtual public icontact_manager2D
{
  public:
    using contact_map = std::unordered_map<contact_key, Contact *>;
    using manager2D<Contact>::manager2D;

    virtual ~contact_manager2D()
    {
        destroy_all_contacts();
    }

    const std::vector<Contact *> &contacts() const
    {
        return this->m_elements;
    }
    const contact_map &contacts_map() const
    {
        return m_unique_contacts;
    }

    using manager2D<Contact>::remove;
    bool remove(const std::size_t index) override
    {
        if (index >= this->m_elements.size())
            return false;
        Contact *contact = this->m_elements[index];
        m_unique_contacts.erase(contact->key());
        destroy_contact(contact);
        this->m_elements.erase(this->m_elements.begin() + index);
        return true;
    }

    std::vector<contact2D *> create_total_contacts_list() const override final
    {
        return std::vector<contact2D *>(this->m_elements.begin(), this->m_elements.end());
    }
    std::vector<contact2D *> create_active_contacts_list() const override final
    {
        std::vector<contact2D *> active_contacts;
        active_contacts.reserve(this->m_elements.size());
        for (Contact *contact : this->m_elements)
            if (contact->enabled()) [[likely]]
                active_contacts.push_back(contact);
        return active_contacts;
    }

    std::size_t total_contacts_count() const override final
    {
        return this->m_elements.size();
    }

    void remove_any_contacts_with(const collider2D *collider) override final
    {
        for (auto it = this->m_elements.begin(); it != this->m_elements.end();)
        {
            Contact *contact = *it;
            if (contact->collider1() == collider || contact->collider2() == collider)
            {
                if (contact->enabled()) [[likely]]
                    contact->on_exit();
                m_unique_contacts.erase(contact->key());
                destroy_contact(contact);
                it = this->m_elements.erase(it);
            }
            else
                ++it;
        }
    }

  protected:
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
        KIT_PERF_SCOPE("ppx::contact_manager2D::remove_expired_contacts")
        std::swap(m_last_contacts, this->m_elements);
        this->m_elements.clear();
        for (Contact *contact : m_last_contacts)
        {
            if (contact->asleep())
            {
                this->m_elements.push_back(contact);
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
            this->m_elements.push_back(contact);
        }
    }

    void create_contact(const contact_key &hash, const collision2D *collision, const std::size_t manifold_index)
    {
        Contact *contact = allocator<Contact>::create(this->world, collision, manifold_index);
        KIT_ASSERT_ERROR(!m_unique_contacts.contains(hash), "Contact already exists!")

        m_unique_contacts.emplace(hash, contact);
        this->m_elements.push_back(contact);
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
        for (Contact *contact : this->m_elements)
        {
            if (contact->enabled()) [[likely]]
                contact->on_exit();
            destroy_contact(contact);
        } // this is expensive, for every contact a call to remove_contact for
          // colliders and bodies. it shouldnt matter because this is almost never
          // called (only when user changes contact solver or disables collisions)
        this->m_elements.clear();
        m_unique_contacts.clear();
    }
};

template <ContactConstraint2D Contact>
class contact_constraint_manager2D final : public contact_manager2D<Contact>, public icontact_constraint_manager2D
{
  public:
    virtual ~contact_constraint_manager2D() = default;
    using contact_manager2D<Contact>::contact_manager2D;

    void startup(std::vector<state2D> &states) override
    {
        m_active_contacts.clear();
        for (Contact *contact : this->m_elements)
        {
            if (!contact->enabled()) [[likely]]
                continue;
            contact->on_pre_solve();
            if (contact->enabled()) [[likely]] // could be disabled by on_pre_solve
            {
                m_active_contacts.push_back(contact);
                contact->startup(states);
            }
        }
    }

    void solve_velocities() override
    {
        for (Contact *contact : m_active_contacts)
            contact->solve_velocities();
    }

    bool solve_positions() override
    {
        bool solved = true;
        for (Contact *contact : m_active_contacts)
            solved &= contact->solve_positions();
        return solved;
    }

    void on_post_solve() override
    {
        for (Contact *contact : m_active_contacts)
            contact->on_post_solve();
    }

  private:
    std::vector<Contact *> m_active_contacts;
};

template <ContactActuator2D Contact>
class contact_actuator_manager2D final : public contact_manager2D<Contact>, public icontact_actuator_manager2D
{
  public:
    virtual ~contact_actuator_manager2D() = default;
    using contact_manager2D<Contact>::contact_manager2D;
    void solve(std::vector<state2D> &states) override
    {
        for (Contact *contact : this->m_elements)
        {
            if (!contact->enabled()) [[likely]]
                continue;

            // on islands, all pre solves are executed before the firs solve ever, but here the implementation differs.
            // take into account for the future :)
            contact->on_pre_solve();
            contact->solve(states);
            contact->on_post_solve();
        }
    }
};
} // namespace ppx