#pragma once

#include "ppx/body/body2D.hpp"
#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/internal/worldref.hpp"
#include "ppx/collision/contacts/contact2D.hpp"
#include "ppx/island/island2D.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/container/hashable_tuple.hpp"
#include "kit/utility/type_constraints.hpp"

namespace ppx
{
class collision_contacts2D : public worldref2D, public kit::toggleable, kit::non_copyable
{
  public:
    using contact_key = contact2D::contact_key;
    collision_contacts2D(world2D &world);
    virtual ~collision_contacts2D() = default;

    virtual void remove_any_contacts_with(const collider2D *collider) = 0;

    virtual std::vector<contact2D *> create_total_contacts_list() const = 0;
    virtual std::vector<contact2D *> create_active_contacts_list() const = 0;

    virtual std::size_t total_contacts_count() const = 0;

    virtual void create_from_collision(const collision2D &collision) = 0;
    virtual void update_from_collision(collision2D &collision) = 0;
    virtual void create_or_update_from_collision(const collision2D &collision) = 0;

    bool checksum() const;
    void inherit(collision_contacts2D &&contacts);

    using kit::toggleable::enabled;
    void enabled(bool enabled) override final;

    std::uint32_t rk_substep_index() const;
    specs::collision_manager2D::contacts2D params;

  private:
    virtual void destroy_all_contacts() = 0;
    virtual void remove_expired_contacts() = 0;

    friend class collision_manager2D;
};
} // namespace ppx
