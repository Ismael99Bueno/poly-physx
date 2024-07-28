#pragma once

#include "ppx/body/body.hpp"
#include "ppx/collision/broad/broad_phase.hpp"
#include "ppx/internal/worldref.hpp"
#include "ppx/collision/contacts/contact.hpp"
#include "ppx/island/island.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/container/hashable_tuple.hpp"
#include "kit/utility/type_constraints.hpp"

namespace ppx
{
class body_manager2D;
class icontact_manager2D : virtual public kit::toggleable
{
  public:
    using contact_key = contact2D::contact_key;

    virtual ~icontact_manager2D() = default;

    virtual void remove_any_contacts_with(const collider2D *collider) = 0;

    virtual std::vector<contact2D *> create_total_contacts_list() const = 0;
    virtual std::vector<contact2D *> create_active_contacts_list() const = 0;

    virtual std::size_t total_contacts_count() const = 0;

    virtual void create_from_collision(const collision2D &collision) = 0;
    virtual void update_from_collision(collision2D &collision) = 0;
    virtual void create_or_update_from_collision(const collision2D &collision) = 0;

    bool checksum(const body_manager2D &bm) const;
    void inherit(icontact_manager2D &&contacts);

    using kit::toggleable::enabled;
    void enabled(bool enabled) override final;

    specs::collision_manager2D::contacts2D params;

  private:
    virtual void destroy_all_contacts() = 0;
    virtual void remove_expired_contacts() = 0;

    friend class collision_manager2D;
};
class icontact_constraint_manager2D : virtual public icontact_manager2D
{
  public:
    virtual void startup(std::vector<state2D> &states) = 0;
    virtual void solve_velocities() = 0;
    virtual bool solve_positions() = 0;
    virtual void on_post_solve() = 0;
};

class icontact_actuator_manager2D : virtual public icontact_manager2D
{
  public:
    virtual void solve(std::vector<state2D> &states) = 0;
};

} // namespace ppx
