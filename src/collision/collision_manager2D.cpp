#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision_manager2D.hpp"

#include "ppx/collision/broad/quad_tree_broad2D.hpp"
#include "ppx/collision/narrow/gjk_epa_narrow2D.hpp"

#include "ppx/collision/contacts/nonpen_contact2D.hpp"

#include "ppx/world2D.hpp"

namespace ppx
{
collision_manager2D::collision_manager2D(world2D &world) : worldref2D(world)
{
    set_broad<quad_tree_broad2D>();
    set_narrow<gjk_epa_narrow2D>();
    set_contact_solver<contact_solver2D<nonpen_contact2D>>();
}

void collision_manager2D::set_constraint_based_contact_solver(contact_constraint_solver2D *solver)
{
    world.joints.constraints.m_contact_solver = solver;
    world.joints.actuators.m_contact_solver = nullptr;
}

void collision_manager2D::set_actuator_based_contact_solver(contact_actuator_solver2D *solver)
{
    world.joints.actuators.m_contact_solver = solver;
    world.joints.constraints.m_contact_solver = nullptr;
}

void collision_manager2D::detect_and_create_contacts()
{
    KIT_PERF_SCOPE("ppx::collision_manager2D::detect_and_create_contacts");
    const auto &pairs = m_broad->enabled() ? m_broad->update_pairs() : m_broad->pairs();
    if (m_narrow->enabled()) [[likely]]
        m_narrow->update_contacts(pairs, m_contacts.get());

    if (m_contacts->enabled()) [[likely]]
        m_contacts->remove_expired_contacts();
}

void collision_manager2D::enabled(const bool enabled)
{
    m_enabled = enabled;
    if (!enabled)
        m_contacts->destroy_all_contacts();
}

} // namespace ppx