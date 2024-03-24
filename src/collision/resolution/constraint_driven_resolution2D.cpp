#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
constraint_driven_resolution2D::constraint_driven_resolution2D(world2D &world, const float slop)
    : collision_resolution2D(world), slop(slop)
{
}

void constraint_driven_resolution2D::update_contacts(const std::vector<collision2D> &collisions)
{
    KIT_PERF_FUNCTION()
    for (const collision2D &collision : collisions)
        for (std::size_t i = 0; i < collision.manifold.size; i++)
        {
            const kit::commutative_tuple<kit::uuid, kit::uuid, std::size_t> hash{collision.collider1->id,
                                                                                 collision.collider2->id, i};
            const auto old_contact = m_contacts.find(hash);
            if (old_contact != m_contacts.end())
                old_contact->second.update(&collision, slop);
            else
                m_contacts.emplace(hash, contact_constraint2D(world, &collision, i, slop));
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

void constraint_driven_resolution2D::solve(const std::vector<collision2D> &collisions)
{
    KIT_ASSERT_ERROR(slop >= 0.f, "Slop must be non-negative: {0}", slop)
    KIT_PERF_SCOPE("Collisions solve")
    update_contacts(collisions);

    {
        KIT_PERF_SCOPE("Startup")
        for (auto &[hash, contact] : m_contacts)
        {
            contact.startup();
            if (world.constraints.warmup)
                contact.warmup();
        }
    }

    {
        KIT_PERF_SCOPE("Solve")
        for (std::size_t i = 0; i < world.constraints.iterations; i++)
            for (auto &[hash, contact] : m_contacts)
                contact.solve();
    }
}
} // namespace ppx