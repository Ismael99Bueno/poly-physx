#include "ppx/internal/pch.hpp"
#include "ppx/events/body_events.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"

namespace ppx
{
body_events::body_events(const std::size_t allocations)
{
    m_collided_ids.reserve(allocations);
}

void body_events::try_enter_or_stay(const collision2D &c) const
{
    if (on_collision_enter.empty() && on_collision_stay.empty())
        return;
    if (m_collided_ids.find(c.body2->id) != m_collided_ids.end())
        on_collision_stay(c);
    else
        on_collision_enter(c);
}
void body_events::try_exit(body2D &current, body2D &outcoming) const
{
    if (on_collision_stay.empty() || m_collided_ids.find(outcoming.id) == m_collided_ids.end())
        return;
    on_collision_exit(current, outcoming);
}
} // namespace ppx