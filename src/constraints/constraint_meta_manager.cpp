#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint_meta_manager.hpp"
#include "ppx/collision/contacts/icontact_manager.hpp"

namespace ppx
{
void constraint_meta_manager2D::solve_velocities(std::vector<state2D> &states)
{
    KIT_PERF_SCOPE("ppx::constraint_meta_manager2D::solve_velocities")
    if (m_contact_solver)
        m_contact_solver->startup(states);

    for (const auto &manager : this->m_elements)
        if (manager->enabled()) [[likely]]
            manager->startup(states);

    for (std::size_t i = 0; i < params.velocity_iterations; i++)
    {
        for (const auto &manager : this->m_elements)
            if (manager->enabled()) [[likely]]
                manager->solve_velocities();
        if (m_contact_solver)
            m_contact_solver->solve_velocities();
    }
}

void constraint_meta_manager2D::solve_positions(std::vector<state2D> &states)
{
    KIT_PERF_SCOPE("ppx::constraint_meta_manager2D::solve_positions")
    for (std::size_t i = 0; i < params.position_iterations; i++)
    {
        bool solved = true;
        for (const auto &manager : this->m_elements)
            if (manager->enabled()) [[likely]]
                solved &= manager->solve_positions();
        if (m_contact_solver)
            solved &= m_contact_solver->solve_positions();
        if (solved)
            break;
    }
}
} // namespace ppx