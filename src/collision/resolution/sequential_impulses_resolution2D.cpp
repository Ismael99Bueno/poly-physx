#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/sequential_impulses_resolution2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void sequential_impulses_resolution2D::startup()
{
    KIT_PERF_SCOPE("Sequential impulses startup")
    m_contacts.startup();
}

void sequential_impulses_resolution2D::solve_velocities()
{
    KIT_PERF_SCOPE("Sequential impulses velocities solve")
    m_contacts.solve_velocities();
}

bool sequential_impulses_resolution2D::solve_positions()
{
    KIT_PERF_SCOPE("Sequential impulses positions solve")
    return m_contacts.solve_positions();
}

void sequential_impulses_resolution2D::on_post_solve()
{
    KIT_PERF_SCOPE("Sequential impulses post solve")
    m_contacts.on_post_solve();
}

void sequential_impulses_resolution2D::remove_any_contacts_with(const collider2D *collider)
{
    KIT_PERF_SCOPE("Sequential impulses remove contacts")
    m_contacts.remove_any_contacts_with(collider);
}

void sequential_impulses_resolution2D::resolve_contacts(const collision_detection2D::collision_map &collisions)
{
    KIT_PERF_SCOPE("Sequential impulses contacts resolve")
    m_contacts.update(collisions);
}

} // namespace ppx