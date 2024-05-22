#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/collision/contacts/contact2D.hpp"

namespace ppx
{
class actuator_driven_resolution2D : public collision_resolution2D
{
  public:
    using collision_resolution2D::collision_resolution2D;

    virtual void solve() = 0;

  protected:
    template <ContactActuator2D T> struct jd_contact_manager : contact_manager<T>
    {
        using contact_manager<T>::contact_manager;
        void solve()
        {
            for (auto &pair : this->contacts)
                pair.second->solve();
        }
    };
};
} // namespace ppx