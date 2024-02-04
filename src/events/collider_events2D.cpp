#include "ppx/internal/pch.hpp"
#include "ppx/events/collider_events2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"

namespace ppx
{
void collider_events2D::try_enter_or_stay(const collision2D &c, const collider2D &outcoming) const
{
    if (on_collision_enter.empty() && on_collision_stay.empty())
        return;
    if (m_collided_ids.find(outcoming.id) != m_collided_ids.end())
        on_collision_stay(c);
    else
        on_collision_enter(c);
}
void collider_events2D::try_exit(collider2D &current, collider2D &outcoming) const
{
    if (on_collision_stay.empty() || m_collided_ids.find(outcoming.id) == m_collided_ids.end())
        return;
    on_collision_exit(current, outcoming);
}
} // namespace ppx