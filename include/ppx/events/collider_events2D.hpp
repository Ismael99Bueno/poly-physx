#pragma once

#include "kit/events/event.hpp"
#include "kit/utility/uuid.hpp"
#include <vector>
#include <functional>
#include <unordered_set>

namespace ppx
{
struct collision2D;
class collider2D;
class collider_events2D
{
  public:
    void try_enter_or_stay(const collision2D &c, const collider2D &outcoming) const;
    void try_exit(collider2D &current, collider2D &outcoming) const;

    kit::event<const collision2D &> on_collision_enter, on_collision_stay;
    kit::event<collider2D &, collider2D &> on_collision_exit;

  private:
    mutable std::unordered_set<kit::uuid> m_collided_ids;

    collider_events2D() = default;
    collider_events2D(const collider_events2D &) = default;
    collider_events2D &operator=(const collider_events2D &) = default;

    friend class collider2D;
};
} // namespace ppx
