#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/collision/contacts/contact2D.hpp"

namespace ppx
{
class constraint_driven_resolution2D : public collision_resolution2D
{
    using collision_resolution2D::collision_resolution2D;

    virtual void startup() = 0;
    virtual void solve_velocities() = 0;
    virtual bool solve_positions() = 0;
    virtual void on_post_solve() = 0;

  protected:
    template <ContactConstraint2D T> struct cd_contact_manager : contact_manager<T>
    {
        using contact_manager<T>::contact_manager;
        void startup()
        {
            for (auto &pair : this->contacts)
            {
                pair.second->startup();
                pair.second->collider1()->events.on_contact_pre_solve(pair.second);
                pair.second->collider2()->events.on_contact_pre_solve(pair.second);
                global_on_contact_pre_solve(world, pair.second);
            }
        }
        void solve_velocities()
        {
            for (auto &pair : this->contacts)
                pair.second->solve_velocities();
        }
        bool solve_positions()
        {
            bool solved = true;
            for (auto &pair : this->contacts)
                solved &= pair.second->solve_positions();
            return solved;
        }
        void on_post_solve()
        {
            for (auto &pair : this->contacts)
            {
                pair.second->collider1()->events.on_contact_post_solve(pair.second);
                pair.second->collider2()->events.on_contact_post_solve(pair.second);
                global_on_contact_post_solve(world, pair.second);
            }
        }
    };

    friend class constraint_meta_manager2D;
};
} // namespace ppx