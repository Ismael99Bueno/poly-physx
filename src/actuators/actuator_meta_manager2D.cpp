#include "ppx/internal/pch.hpp"
#include "ppx/actuators/actuator_meta_manager2D.hpp"
#include "ppx/collision/contacts/icontact_manager2D.hpp"

namespace ppx
{
void actuator_meta_manager2D::solve(std::vector<state2D> &states)
{
    KIT_PERF_SCOPE("ppx::actuator_meta_manager2D::solve")
    if (m_contact_solver)
        m_contact_solver->solve(states);
    for (const auto &manager : m_elements)
        if (manager->enabled()) [[likely]]
            manager->solve(states);
}
} // namespace ppx