#pragma once

#include "ppx/body/body2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/internal/worldref.hpp"
#include "ppx/collision/contacts/contact2D.hpp"
#include "ppx/island/island2D.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/container/hashable_tuple.hpp"
#include "kit/utility/type_constraints.hpp"

namespace ppx
{
class collision_contacts2D : public worldref2D, kit::non_copyable
{
  public:
    using contact_key = kit::non_commutative_tuple<const collider2D *, const collider2D *, std::uint32_t>;

    collision_contacts2D(world2D &world);
    virtual ~collision_contacts2D() = default;

    specs::collision_manager2D::contacts2D params;

    virtual void remove_any_contacts_with(const collider2D *collider) = 0;

    virtual std::vector<contact2D *> create_contacts_list() const = 0;
    virtual std::size_t size() const = 0;

    bool checksum() const;
    void inherit(collision_contacts2D &&contacts);

    bool enabled() const;
    void enabled(bool enable);

  private:
    bool m_enabled = true;

    virtual void on_attach()
    {
    }

    virtual void destroy_all_contacts() = 0;
    virtual void create_contacts_from_collisions(const collision_detection2D::collision_map &collisions) = 0;

    friend class collision_manager2D;
};
} // namespace ppx
