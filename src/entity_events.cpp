#include "ppx/entity_events.hpp"
#include "ppx/collider2D.hpp"

namespace ppx
{
    entity_events::entity_events(const std::size_t allocations) { m_collided_ids.reserve(allocations); }

    void entity_events::try_enter_or_stay(const collision2D &c) const
    {
        if (m_processed || (on_collision_enter.subscriptions().empty() &&
                            on_collision_stay.subscriptions().empty()))
            return;
        if (m_collided_ids.find(c.incoming.id()) != m_collided_ids.end())
            on_collision_stay(c);
        else
            on_collision_enter(c);
        m_processed = true;
    }
    void entity_events::try_exit(const entity2D_ptr &incoming) const
    {
        if (m_processed || on_collision_stay.subscriptions().empty() || m_collided_ids.find(incoming.id()) == m_collided_ids.end())
            return;
        on_collision_exit(incoming);
    }
    void entity_events::reset() { m_processed = false; }
}