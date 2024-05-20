#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/collision/contacts/contact_joint2D.hpp"

namespace ppx
{
class joint_driven_resolution2D : public collision_resolution2D
{
  public:
    using collision_resolution2D::collision_resolution2D;

    virtual void solve() = 0;

  protected:
    template <kit::DerivedFrom<contact_joint2D> T> struct jd_contact_manager : contact_manager<T>
    {
        using contact_manager<T>::contact_manager;
        contact_manager<T>::contact_map contacts;
        void solve()
        {
            for (auto &pair : contacts)
                pair.second->solve();
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
};
} // namespace ppx