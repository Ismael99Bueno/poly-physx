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
    KIT_ASSERT_ERROR(this->world.constraints.baumgarte_coef >= 0.f, "Baumgarte coef must be non-negative: {0}",
                     this->world.constraints.baumgarte_coef)
    KIT_ASSERT_ERROR(this->world.constraints.baumgarte_threshold >= 0.f,
                     "Baumgarte threshold must be non-negative: {0}", this->world.constraints.baumgarte_threshold)

    KIT_PERF_SCOPE("Constraints solve")
    if (m_si_solver)
        m_si_solver->startup();
    for (const auto &solver : this->m_elements)
        solver->startup();

    for (std::size_t i = 0; i < this->world.constraints.velocity_iterations; i++)
    {
        if (m_si_solver)
            m_si_solver->solve_contacts();
        for (const auto &solver : this->m_elements)
            solver->solve();
    }
    for (std::size_t i = 0; i < this->world.constraints.position_iterations; i++)
    {
        bool fully_adjusted = true;
        for (const auto &solver : this->m_elements)
            fully_adjusted &= solver->adjust_positions();
        if (m_si_solver)
            fully_adjusted &= m_si_solver->adjust_positions();
        if (fully_adjusted)
            break;
    }

    m_si_solver = nullptr;
}

} // namespace ppx