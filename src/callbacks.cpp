#include "callbacks.hpp"

namespace ppx
{
    callbacks::callbacks(engine_key) {}

    void callbacks::on_entity_addition(const add_entity &on_add) { m_on_entity_addition.push_back(on_add); }
    void callbacks::on_early_entity_removal(const early_remove_entity &on_remove) { m_on_early_entity_removal.push_back(on_remove); }
    void callbacks::on_late_entity_removal(const late_remove_entity &on_remove) { m_on_late_entity_removal.push_back(on_remove); }

    void callbacks::on_spring_addition(const add_spring &on_add) { m_on_spring_addition.push_back(on_add); }
    void callbacks::on_spring_removal(const remove_spring &on_remove) { m_on_spring_removal.push_back(on_remove); }

    void callbacks::on_constraint_addition(const constraint_cb &on_add) { m_on_constraint_addition.push_back(on_add); }
    void callbacks::on_constraint_removal(const constraint_cb &on_remove) { m_on_constraint_removal.push_back(on_remove); }

    void callbacks::entity_addition(entity2D_ptr e) const
    {
        for (const auto &cb : m_on_entity_addition)
            cb(e);
    }
    void callbacks::early_entity_removal(entity2D &e) const
    {
        for (const auto &cb : m_on_early_entity_removal)
            cb(e);
    }
    void callbacks::late_entity_removal(std::size_t index) const
    {
        for (const auto &cb : m_on_late_entity_removal)
            cb(index);
    }

    void callbacks::spring_addition(spring2D *sp) const
    {
        for (const auto &cb : m_on_spring_addition)
            cb(sp);
    }
    void callbacks::spring_removal(spring2D &sp) const
    {
        for (const auto &cb : m_on_spring_removal)
            cb(sp);
    }

    void callbacks::constraint_addition(const std::shared_ptr<constraint_interface2D> &ctr) const
    {
        for (const auto &cb : m_on_constraint_addition)
            cb(ctr);
    }
    void callbacks::constraint_removal(const std::shared_ptr<constraint_interface2D> &ctr) const
    {
        for (const auto &cb : m_on_constraint_removal)
            cb(ctr);
    }
}