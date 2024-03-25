#include "ppx/internal/pch.hpp"
#include "ppx/events/collider_events2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"

namespace ppx
{
bool collider_events2D::empty() const
{
    return on_collision_enter.empty() && on_collision_pre_solve.empty() && on_collision_post_solve.empty() &&
           on_collision_exit.empty();
}
} // namespace ppx