#include "ppx/internal/pch.hpp"
#include "ppx/joints/meta_manager2D.hpp"
#include "ppx/collision/resolution/sequential_impulses_resolution2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
template <IManager IM> bool meta_manager2D<IM>::remove(std::size_t index)
{
    if (index >= this->m_elements.size())
        return false;
    this->m_elements.erase(this->m_elements.begin() + index);
    return true;
}

void joint_meta_manager2D::solve()
{
    KIT_PERF_SCOPE("Joints solve")
    for (const auto &solver : m_elements)
        solver->solve();
}

void constraint_meta_manager2D::delegate_contacts_resolution(sequential_impulses_resolution2D *solver)
{
    m_si_solver = solver;
}

void constraint_meta_manager2D::solve()
{
    KIT_PERF_SCOPE("Constraints solve")
    const bool warmup = world.constraints.warmup;
    const std::size_t viters = world.constraints.velocity_iterations;
    const std::size_t piters = world.constraints.position_iterations;
    if (m_si_solver)
    {
        m_si_solver->startup();
        if (warmup)
            m_si_solver->warmup();
    }
    for (const auto &solver : this->m_elements)
        solver->startup();
    if (warmup)
        for (const auto &solver : this->m_elements)
            solver->warmup();

    for (std::size_t i = 0; i < viters; i++)
    {
        if (m_si_solver)
            m_si_solver->solve_contacts();
        for (const auto &solver : this->m_elements)
            solver->solve();
    }
    for (std::size_t i = 0; i < piters; i++)
    {
        bool fully_adjusted = true;
        for (const auto &solver : this->m_elements)
        {
            if (i > 0)
                solver->startup();
            fully_adjusted &= solver->adjust_positions();
        }
        if (m_si_solver)
        {
            if (i > 0)
                m_si_solver->startup();
            fully_adjusted &= m_si_solver->adjust_positions();
        }
        if (fully_adjusted)
            break;
    }

    m_si_solver = nullptr;
}

template class meta_manager2D<ijoint_manager2D>;
template class meta_manager2D<iconstraint_manager2D>;

} // namespace ppx