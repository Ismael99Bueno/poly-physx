#include "ppx/internal/pch.hpp"
#include "ppx/events/collider_events2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"

namespace ppx
{
bool collider_events2D::just_collided(const collision2D &c, const collider2D &incoming)
{
    return m_collided_ids.insert(incoming.id).second;
}
bool collider_events2D::just_separated(const collider2D &current, const collider2D &incoming)
{
    return m_collided_ids.erase(incoming.id) == 1;
}
} // namespace ppx