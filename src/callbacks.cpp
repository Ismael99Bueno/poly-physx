#include "callbacks.hpp"

namespace ppx
{
    callbacks::callbacks(engine_key) {}

    void callbacks::on_entity_addition(const entity_cb &on_add) { m_on_entity_addition.push_back(on_add); }
    void callbacks::on_early_entity_removal(const entity_cb &on_remove) { m_on_early_entity_removal.push_back(on_remove); }
    void callbacks::on_late_entity_removal(const late_remove_entity_cb &on_remove) { m_on_late_entity_removal.push_back(on_remove); }

    void callbacks::on_spring_addition(const spring_cb &on_add) { m_on_spring_addition.push_back(on_add); }
    void callbacks::on_spring_removal(const spring_cb &on_remove) { m_on_spring_removal.push_back(on_remove); }

    void callbacks::on_constraint_addition(const constraint_cb &on_add) { m_on_constraint_addition.push_back(on_add); }
    void callbacks::on_constraint_removal(const constraint_cb &on_remove) { m_on_constraint_removal.push_back(on_remove); }

    void callbacks::entity_addition(engine_key, entity2D_ptr e) const
    {
        for (const auto &cb : m_on_entity_addition)
            cb(e);
    }
    void callbacks::early_entity_removal(engine_key, entity2D_ptr e) const
    {
        for (const auto &cb : m_on_early_entity_removal)
            cb(e);
    }
    void callbacks::late_entity_removal(engine_key, std::size_t index) const
    {
        for (const auto &cb : m_on_late_entity_removal)
            cb(index);
    }

    void callbacks::spring_addition(engine_key, spring2D *sp) const
    {
        for (const auto &cb : m_on_spring_addition)
            cb(sp);
    }
    void callbacks::spring_removal(engine_key, spring2D *sp) const
    {
        for (const auto &cb : m_on_spring_removal)
            cb(sp);
    }

    void callbacks::constraint_addition(engine_key, const std::shared_ptr<constraint_interface2D> &ctr) const
    {
        for (const auto &cb : m_on_constraint_addition)
            cb(ctr);
    }
    void callbacks::constraint_removal(engine_key, const std::shared_ptr<constraint_interface2D> &ctr) const
    {
        for (const auto &cb : m_on_constraint_removal)
            cb(ctr);
    }
}