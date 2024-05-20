#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/collision/contacts/contact_constraint2D.hpp"

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
    template <kit::DerivedFrom<contact_constraint2D> T> struct cd_contact_manager : contact_manager<T>
    {
        using contact_manager<T>::contact_manager;
        contact_manager<T>::contact_map contacts;
        void startup()
        {
            for (auto &pair : contacts)
            {
                pair.second->startup();
                pair.second->collider1()->events.on_contact_pre_solve(pair.second);
                pair.second->collider2()->events.on_contact_pre_solve(pair.second);
                world.collisions.events.on_contact_pre_solve(pair.second);
            }
        }
        void solve_velocities()
        {
            for (auto &pair : contacts)
                pair.second->solve_velocities();
        }
        bool solve_positions()
        {
            bool fully_adjusted = true;
            for (auto &pair : contacts)
                fully_adjusted &= pair.second->solve_positions();
            return fully_adjusted;
        }
        void on_post_solve()
        {
            for (auto &pair : contacts)
            {
                pair.second->collider1()->events.on_contact_post_solve(pair.second);
                pair.second->collider2()->events.on_contact_post_solve(pair.second);
                world.collisions.events.on_contact_post_solve(pair.second);
            }
        }

        void update(const collision_detection2D::collision_map &collisions)
        {
            contact_manager<T>::update(collisions, contacts);
        }
        void remove_any_contacts_with(const collider2D *collider)
        {
            contact_manager<T>::remove_any_contacts_with(collider, contacts);
        }
    };

    friend class constraint_meta_manager2D;
};
} // namespace ppx