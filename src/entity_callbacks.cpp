#include "entity_callbacks.hpp"
#include "collider2D.hpp"

namespace ppx
{
    entity_callbacks::entity_callbacks(entity_key, const std::size_t allocations) { m_collided_ids.reserve(allocations); }

    void entity_callbacks::on_collision_enter(const enter_stay_cb &on_enter) { m_on_enter.push_back(on_enter); }
    void entity_callbacks::on_collision_stay(const enter_stay_cb &on_stay) { m_on_stay.push_back(on_stay); }
    void entity_callbacks::on_collision_exit(const exit_cb &on_exit) { m_on_exit.push_back(on_exit); }

    void entity_callbacks::try_enter_or_stay(const collision2D &c) const
    {
        if (m_processed || (m_on_enter.empty() && m_on_stay.empty()))
            return;
        if (m_collided_ids.find(c.incoming.id()) != m_collided_ids.end())
            on_stay(c);
        else
            on_enter(c);
        m_processed = true;
    }
    void entity_callbacks::try_exit(const entity2D_ptr &incoming) const
    {
        if (m_processed || m_on_stay.empty() || m_collided_ids.find(incoming.id()) == m_collided_ids.end())
            return;
        on_exit(incoming);
    }
    void entity_callbacks::reset(engine_key) { m_processed = false; }

    void entity_callbacks::on_enter(const collision2D &c) const
    {
        call(m_on_enter, c);
        m_collided_ids.insert(c.incoming.id());
    }
    void entity_callbacks::on_stay(const collision2D &c) const { call(m_on_stay, c); }
    void entity_callbacks::on_exit(const entity2D_ptr &incoming) const
    {
        call(m_on_exit, incoming);
        m_collided_ids.erase(incoming.id());
    }
}