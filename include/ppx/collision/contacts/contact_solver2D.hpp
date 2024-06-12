#pragma once

#include "ppx/collision/contacts/contact_manager2D.hpp"

namespace ppx
{
class contact_constraint_solver2D
{
  public:
    virtual ~contact_constraint_solver2D() = default;

    virtual void startup() = 0;
    virtual void solve_velocities() = 0;
    virtual bool solve_positions() = 0;
    virtual void on_post_solve() = 0;
};

class contact_actuator_solver2D
{
  public:
    virtual ~contact_actuator_solver2D() = default;

    virtual void solve() = 0;
};

template <Contact2D Contact> class contact_solver2D
{
};

template <ContactConstraint2D Contact>
class contact_solver2D<Contact> : public contact_manager2D<Contact>, public contact_constraint_solver2D
{
  public:
    using contact_manager2D<Contact>::contact_manager2D;

    void startup() override
    {
        for (auto &pair : this->m_contacts)
        {
            if (!pair.second->enabled) // no need to check sleep: if using this, islands are disabled
                continue;
            pair.second->on_pre_solve();
            pair.second->startup();
        }
    }

    void solve_velocities() override
    {
        for (auto &pair : this->m_contacts)
            if (pair.second->enabled)
                pair.second->solve_velocities();
    }

    bool solve_positions() override
    {
        bool solved = true;
        for (auto &pair : this->m_contacts)
            if (pair.second->enabled)
                solved &= pair.second->solve_positions();
        return solved;
    }

    void on_post_solve() override
    {
        for (auto &pair : this->m_contacts)
            if (pair.second->enabled)
                pair.second->on_post_solve();
    }
};

template <ContactActuator2D Contact>
class contact_solver2D<Contact> : public contact_manager2D<Contact>, public contact_actuator_solver2D
{
  public:
    using contact_manager2D<Contact>::contact_manager2D;

    void solve() override
    {
        for (auto &pair : this->m_contacts)
        {
            if (!pair.second->enabled)
                continue;

            // on islands, all pre solves are executed before the firs solve ever, but here the implementation differs.
            // take into account for the future :)
            pair.second->on_pre_solve();
            pair.second->solve();
            pair.second->on_post_solve();
        }
    }
};

} // namespace ppx