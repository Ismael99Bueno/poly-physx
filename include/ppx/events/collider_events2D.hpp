#pragma once

#include "kit/events/event.hpp"
#include "kit/utility/uuid.hpp"
#include <vector>
#include <functional>

namespace ppx
{
struct collision2D;
class collider2D;
class collider_events2D
{
  public:
    kit::event<const collision2D &> on_collision_enter, on_collision_pre_solve, on_collision_post_solve;
    kit::event<collider2D *, collider2D *> on_collision_exit;

    bool empty() const;

  private:
    collider_events2D() = default;
    collider_events2D(const collider_events2D &) = default;
    collider_events2D &operator=(const collider_events2D &) = default;

    friend class collider2D;
    friend class collision_detection2D;
};
} // namespace ppx
