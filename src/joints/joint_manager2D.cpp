#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
joint_solver2D::joint_solver2D(const std::string &name) : kit::identifiable<std::string>(name)
{
}

void joint_meta_manager2D::solve()
{
    KIT_PERF_SCOPE("Joints solve")
    for (const auto &solver : m_elements)
        solver->solve();
}

void joint_meta_manager2D::validate()
{
    for (const auto &solver : m_elements)
        solver->validate();
}

} // namespace ppx