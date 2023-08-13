#include "ppx/internal/pch.hpp"
#include "ppx/events/body_events.hpp"
#include "ppx/collision/collider2D.hpp"

namespace ppx
{
body_events::body_events(const std::size_t allocations)
{
    m_collided_ids.reserve(allocations);
}

void body_events::try_enter_or_stay(const collision2D &c) const
{
    if (on_collision_enter.callbacks().empty() && on_collision_stay.callbacks().empty())
        return;
    if (m_collided_ids.find(c.incoming->id) != m_collided_ids.end())
        on_collision_stay(c);
    else
        on_collision_enter(c);
}
void body_events::try_exit(const body2D::ptr &incoming) const
{
    if (on_collision_stay.callbacks().empty() || m_collided_ids.find(incoming->id) == m_collided_ids.end())
        return;
    on_collision_exit(incoming);
}
} // namespace ppx