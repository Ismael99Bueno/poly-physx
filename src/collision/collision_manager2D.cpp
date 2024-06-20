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
    set_cp_narrow<gjk_epa_narrow2D>();
    set_pp_narrow<gjk_epa_narrow2D>();
    set_contact_solver<contact_solver2D<nonpen_contact2D>>();
}

const collision2D &collision_manager2D::operator[](
    const kit::commutative_tuple<const collider2D *, const collider2D *> &key) const
{
    return m_broad->collisions().at(key);
}
const collision2D &collision_manager2D::at(
    const kit::commutative_tuple<const collider2D *, const collider2D *> &key) const
{
    return m_broad->collisions().at(key);
}
const collision2D &collision_manager2D::at(const collider2D *collider1, const collider2D *collider2) const
{
    return m_broad->collisions().at({collider1, collider2});
}

bool collision_manager2D::contains(const kit::commutative_tuple<const collider2D *, const collider2D *> &key) const
{
    return m_broad->collisions().contains(key);
}
bool collision_manager2D::contains(const collider2D *collider1, const collider2D *collider2) const
{
    return m_broad->collisions().contains({collider1, collider2});
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
    KIT_PERF_FUNCTION()
    if (!m_broad->enabled)
        return;
    const auto &collisions = m_broad->detect_collisions_cached(m_cp_narrow.get(), m_pp_narrow.get());
    if (m_contacts->enabled())
        m_contacts->create_contacts_from_collisions(collisions);
}

std::size_t collision_manager2D::size() const
{
    return m_broad->collisions().size();
}
bool collision_manager2D::empty() const
{
    return m_broad->collisions().empty();
}

bool collision_manager2D::enabled() const
{
    return m_enabled;
}
void collision_manager2D::enabled(const bool enabled)
{
    m_enabled = enabled;
    if (!enabled)
        m_contacts->destroy_all_contacts();
}

} // namespace ppx