#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/sequential_impulses_resolution2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void sequential_impulses_resolution2D::update_contacts(const collision_detection2D::collision_map &collisions)
{
    KIT_PERF_FUNCTION()
    for (const auto &pair : collisions)
    {
        const collision2D &collision = pair.second;
        if (!collision.collided || !collision.enabled)
            continue;
        for (std::size_t i = 0; i < collision.manifold.size(); i++)
        {
            const kit::non_commutative_tuple<const collider2D *, const collider2D *, std::uint32_t> hash{
                collision.collider1, collision.collider2, collision.manifold[i].id.key};
            const auto old_contact = m_contacts.find(hash);
            if (old_contact != m_contacts.end())
                old_contact->second.update(&collision, i);
            else
                m_contacts.emplace(hash, contact_constraint2D(world, &collision, i));
        }
    }
    for (auto it = m_contacts.begin(); it != m_contacts.end();)
        if (!it->second.recently_updated)
            it = m_contacts.erase(it);
        else
        {
            it->second.recently_updated = false;
            ++it;
        }
}

void sequential_impulses_resolution2D::solve(const collision_detection2D::collision_map &collisions)
{
    KIT_PERF_SCOPE("Collisions solve")
    update_contacts(collisions);
    world.joints.constraint_based.delegate_contacts_resolution(this);
}

void sequential_impulses_resolution2D::startup()
{
    KIT_PERF_SCOPE("Contacts startup")
    for (auto &[hash, contact] : m_contacts)
        contact.startup();
}

void sequential_impulses_resolution2D::solve_velocities()
{
    KIT_PERF_SCOPE("Contacts solve")
    for (auto &[hash, contact] : m_contacts)
        contact.solve_velocities();
}

bool sequential_impulses_resolution2D::solve_positions()
{
    KIT_PERF_SCOPE("Contacts adjust positions")
    bool fully_adjusted = true;
    for (auto &[hash, contact] : m_contacts)
        fully_adjusted &= contact.solve_positions();
    return fully_adjusted;
}

} // namespace ppx