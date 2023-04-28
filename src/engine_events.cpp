#include "ppx/engine_events.hpp"

namespace ppx
{
    engine_events::engine_events(engine_key) {}

    void engine_events::on_entity_addition(const add_entity &on_add) { m_on_entity_addition.push_back(on_add); }
    void engine_events::on_early_entity_removal(const early_remove_entity &on_remove) { m_on_early_entity_removal.push_back(on_remove); }
    void engine_events::on_late_entity_removal(const late_remove_entity &on_remove) { m_on_late_entity_removal.push_back(on_remove); }

    void engine_events::on_spring_addition(const add_spring &on_add) { m_on_spring_addition.push_back(on_add); }
    void engine_events::on_spring_removal(const remove_spring &on_remove) { m_on_spring_removal.push_back(on_remove); }

    void engine_events::on_constraint_addition(const constraint_cb &on_add) { m_on_constraint_addition.push_back(on_add); }
    void engine_events::on_constraint_removal(const constraint_cb &on_remove) { m_on_constraint_removal.push_back(on_remove); }

    void engine_events::entity_addition(const entity2D_ptr &e) const
    {
        for (const auto &cb : m_on_entity_addition)
            cb(e);
    }
    void engine_events::early_entity_removal(entity2D &e) const
    {
        for (const auto &cb : m_on_early_entity_removal)
            cb(e);
    }
    void engine_events::late_entity_removal(std::size_t index) const
    {
        for (const auto &cb : m_on_late_entity_removal)
            cb(index);
    }

    void engine_events::spring_addition(spring2D *sp) const
    {
        for (const auto &cb : m_on_spring_addition)
            cb(sp);
    }
    void engine_events::spring_removal(spring2D &sp) const
    {
        for (const auto &cb : m_on_spring_removal)
            cb(sp);
    }

    void engine_events::constraint_addition(const std::shared_ptr<constraint_interface2D> &ctr) const
    {
        for (const auto &cb : m_on_constraint_addition)
            cb(ctr);
    }
    void engine_events::constraint_removal(const std::shared_ptr<constraint_interface2D> &ctr) const
    {
        for (const auto &cb : m_on_constraint_removal)
            cb(ctr);
    }
}