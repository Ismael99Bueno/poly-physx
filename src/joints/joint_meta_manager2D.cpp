#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_meta_manager2D.hpp"
#include "ppx/collision/contacts/contact_solver2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
template <IManager IM> bool joint_meta_manager2D<IM>::remove(std::size_t index)
{
    if (index >= this->m_elements.size())
        return false;
    this->m_elements.erase(this->m_elements.begin() + index);
    return true;
}

template <IManager IM> bool joint_meta_manager2D<IM>::remove(joint2D *joint)
{
    for (const auto &manager : this->m_elements)
        if (manager->remove(joint))
            return true;
    return false;
}

void actuator_meta_manager2D::solve(std::vector<state2D> &states)
{
    KIT_PERF_SCOPE("ppx::actuator_meta_manager2D::solve")
    if (m_contact_solver)
        m_contact_solver->solve(states);
    for (const auto &manager : m_elements)
        if (manager->enabled()) [[likely]]
            manager->solve(states);
}

void constraint_meta_manager2D::solve(std::vector<state2D> &states)
{
    KIT_PERF_SCOPE("ppx::constraint_meta_manager2D::solve")
    if (m_contact_solver)
        m_contact_solver->startup(states);

    for (const auto &manager : this->m_elements)
        if (manager->enabled()) [[likely]]
            manager->startup(states);

    for (std::size_t i = 0; i < params.velocity_iterations; i++)
    {
        if (m_contact_solver)
            m_contact_solver->solve_velocities();
        for (const auto &manager : this->m_elements)
            if (manager->enabled()) [[likely]]
                manager->solve_velocities();
    }
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

template class joint_meta_manager2D<iactuator_manager2D>;
template class joint_meta_manager2D<iconstraint_manager2D>;

} // namespace ppx