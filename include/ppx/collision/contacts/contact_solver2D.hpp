#pragma once

#include "ppx/collision/contacts/contact_manager2D.hpp"

namespace ppx
{
class contact_constraint_solver2D
{
  public:
    virtual ~contact_constraint_solver2D() = default;

    virtual void startup(std::vector<state2D> &states) = 0;
    virtual void solve_velocities() = 0;
    virtual bool solve_positions() = 0;
    virtual void on_post_solve() = 0;
};

class contact_actuator_solver2D
{
  public:
    virtual ~contact_actuator_solver2D() = default;

    virtual void solve(std::vector<state2D> &states) = 0;
};

template <Contact2D Contact> class contact_solver2D;

template <ContactConstraint2D Contact>
class contact_solver2D<Contact> final : public contact_manager2D<Contact>, public contact_constraint_solver2D
{
  public:
    using contact_manager2D<Contact>::contact_manager2D;

    void startup(std::vector<state2D> &states) override
    {
        m_active_contacts.clear();
        for (Contact *contact : this->m_contacts)
        {
            if (!contact->enabled()) [[likely]] // no need to check sleep: if using this, islands are disabled
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
class contact_solver2D<Contact> final : public contact_manager2D<Contact>, public contact_actuator_solver2D
{
  public:
    using contact_manager2D<Contact>::contact_manager2D;

    void solve(std::vector<state2D> &states) override
    {
        for (Contact *contact : this->m_contacts)
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