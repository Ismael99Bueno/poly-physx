#include "entity_callbacks.hpp"

namespace ppx
{
    entity_callbacks::entity_callbacks(entity_key) {}

    void entity_callbacks::on_collision_enter(const enter_stay_cb &on_enter) { m_on_enter.push_back(on_enter); }
    void entity_callbacks::on_collision_stay(const enter_stay_cb &on_stay) { m_on_stay.push_back(on_stay); }
    void entity_callbacks::on_collision_exit(const exit_cb &on_exit) { m_on_exit.push_back(on_exit); }

    void entity_callbacks::try_enter_or_stay(const collision2D &c) const
    {
        if (m_processed)
            return;
        if (m_in_collision)
            on_stay(c);
        else
            on_enter(c);
        m_processed = true;
    }
    void entity_callbacks::try_exit(const entity2D_ptr &other) const
    {
        if (m_processed)
            return;
        if (m_in_collision)
            on_exit(other);
    }
    void entity_callbacks::reset(engine_key) { m_processed = false; }

    void entity_callbacks::on_enter(const collision2D &c) const
    {
        call(m_on_enter, c);
        m_in_collision = true;
    }
    void entity_callbacks::on_stay(const collision2D &c) const { call(m_on_stay, c); }
    void entity_callbacks::on_exit(const entity2D_ptr &other) const
    {
        call(m_on_exit, other);
        m_in_collision = false;
    }
}