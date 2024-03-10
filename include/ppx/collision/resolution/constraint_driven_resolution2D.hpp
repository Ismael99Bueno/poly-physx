#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "kit/container/hashable_tuple.hpp"

namespace ppx
{
class constraint_driven_resolution2D : public collision_resolution2D
{
  public:
    constraint_driven_resolution2D(world2D &world, float slop = 0.f);

    float slop;

  private:
    void update_contacts(const std::vector<collision2D> &collisions);
    void solve(const std::vector<collision2D> &collisions) override;

    std::unordered_map<kit::commutative_tuple<kit::uuid, kit::uuid, std::size_t>, contact_constraint2D> m_contacts;
};
} // namespace ppx
