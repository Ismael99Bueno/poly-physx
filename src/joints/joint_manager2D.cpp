#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
joint_solver2D::joint_solver2D(const kit::uuid &id) : kit::identifiable<>(id)
{
}

joint_repository2D::joint_repository2D(world2D &world) : manager2D<kit::scope<joint_solver2D>>(world)
{
}

bool joint_repository2D::remove(std::size_t index)
{
    if (index >= m_elements.size())
        return false;

    m_elements.erase(m_elements.begin() + index);
    return true;
}

void joint_repository2D::solve() const
{
    KIT_ASSERT_ERROR(world.constraints.baumgarte_coef >= 0.f, "Baumgarte coef must be non-negative: {0}",
                     world.constraints.baumgarte_coef)
    KIT_ASSERT_ERROR(world.constraints.baumgarte_threshold >= 0.f, "Baumgarte threshold must be non-negative: {0}",
                     world.constraints.baumgarte_threshold)
    KIT_ASSERT_ERROR(world.constraints.iterations > 0, "Iterations must be positive: {0}", world.constraints.iterations)
    KIT_PERF_SCOPE("Joints solve")
    for (const auto &solver : m_elements)
        solver->solve();
}

void joint_repository2D::validate() const
{
    for (const auto &solver : m_elements)
        solver->validate();
}

} // namespace ppx