#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "kit/container/hashable_tuple.hpp"

namespace ppx
{
class sequential_impulses_resolution2D : public collision_resolution2D
{
  public:
    using contact_map = std::unordered_map<kit::commutative_tuple<const collider2D *, const collider2D *, std::size_t>,
                                           contact_constraint2D>;
    using collision_resolution2D::collision_resolution2D;

  private:
    void update_contacts(const collision_detection2D::collision_map &collisions);
    void solve(const collision_detection2D::collision_map &collisions) override;

    void startup();
    void solve_contacts();
    bool adjust_positions();

    contact_map m_contacts;

    friend class constraint_meta_manager2D;
};
} // namespace ppx
