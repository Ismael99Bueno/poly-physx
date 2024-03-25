#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "kit/container/hashable_tuple.hpp"

namespace ppx
{
class sequential_impulses_resolution2D : public collision_resolution2D
{
  public:
    sequential_impulses_resolution2D(world2D &world, float slop = 0.f);

    float slop;

  private:
    void update_contacts(const collision_detection2D::collision_map &collisions);
    void solve(const collision_detection2D::collision_map &collisions) override;

    std::unordered_map<kit::commutative_tuple<const collider2D *, const collider2D *, std::size_t>,
                       contact_constraint2D>
        m_contacts;
};
} // namespace ppx
