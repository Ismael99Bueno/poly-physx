#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/spring_driven_resolution2D.hpp"

#include "kit/multithreading/mt_for_each.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
void spring_driven_resolution2D::solve()
{
    KIT_PERF_SCOPE("Spring driven contacts solve")
    m_contacts.solve();
}

void spring_driven_resolution2D::remove_any_contacts_with(const collider2D *collider)
{
    KIT_PERF_SCOPE("Sequential impulses remove contacts")
    m_contacts.remove_any_contacts_with(collider);
}

void spring_driven_resolution2D::resolve_contacts(const collision_detection2D::collision_map &collisions)
{
    KIT_ASSERT_ERROR(sd_contact2D::rigidity >= 0.f, "Rigidity must be non-negative: {0}", sd_contact2D::rigidity)
    KIT_ASSERT_ERROR(sd_contact2D::max_tangent_damping >= 0.f, "Tangent damping must be non-negative: {0}",
                     sd_contact2D::max_tangent_damping)
    KIT_ASSERT_ERROR(sd_contact2D::max_normal_damping >= 0.f, "Normal damping must be non-negative: {0}",
                     sd_contact2D::max_normal_damping)

    KIT_PERF_SCOPE("Spring driven contacts resolve")
    m_contacts.update(collisions);
}
} // namespace ppx