#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
constraint_solver2D::constraint_solver2D(const std::string &name) : kit::identifiable<std::string>(name)
{
}

void constraint_meta_manager2D::startup()
{
    for (auto &solver : m_elements)
        solver->startup();
}
void constraint_meta_manager2D::solve()
{
    KIT_ASSERT_ERROR(world.constraints.baumgarte_coef >= 0.f, "Baumgarte coef must be non-negative: {0}",
                     world.constraints.baumgarte_coef)
    KIT_ASSERT_ERROR(world.constraints.baumgarte_threshold >= 0.f, "Baumgarte threshold must be non-negative: {0}",
                     world.constraints.baumgarte_threshold)
    KIT_ASSERT_ERROR(world.constraints.iterations > 0, "Iterations must be positive: {0}", world.constraints.iterations)
    KIT_PERF_SCOPE("Constraints solve")
    for (const auto &solver : m_elements)
        solver->solve();
}

void constraint_meta_manager2D::validate()
{
    for (const auto &solver : m_elements)
        solver->validate();
}

} // namespace ppx