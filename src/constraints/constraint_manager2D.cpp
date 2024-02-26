#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
void constraint_manager2D::process_addition(kit::scope<constraint2D> &&ctr)
{
    m_elements.push_back(std::move(ctr));
    events.on_addition(m_elements.back().get());
}

bool constraint_manager2D::remove(std::size_t index)
{
    if (index >= m_elements.size())
        return false;

    events.on_removal(*m_elements[index]);
    m_elements.erase(m_elements.begin() + index);
    return true;
}

template <typename T>
static std::vector<T> from_ids(const std::vector<kit::uuid> &ids,
                               const std::vector<kit::scope<constraint2D>> &constraints)
{
    KIT_ASSERT_ERROR(std::unordered_set<kit::uuid>(ids.begin(), ids.end()).size() == ids.size(),
                     "IDs list must not contain duplicates!")

    std::vector<T> found_ctrs;
    if (ids.empty())
        return found_ctrs;

    found_ctrs.reserve(constraints.size());
    for (const auto &ctr : constraints)
    {
        bool found_match = true;
        for (kit::uuid id : ids)
            if (!ctr->contains(id))
            {
                found_match = false;
                break;
            }
        if (found_match)
            found_ctrs.push_back(ctr.get());
    }
    return found_ctrs;
}

std::vector<const constraint2D *> constraint_manager2D::operator[](const std::vector<kit::uuid> &ids) const
{
    return from_ids<const constraint2D *>(ids, m_elements);
}
std::vector<constraint2D *> constraint_manager2D::operator[](const std::vector<kit::uuid> &ids)
{
    return from_ids<constraint2D *>(ids, m_elements);
}

void constraint_manager2D::validate()
{
    for (auto it = m_elements.begin(); it != m_elements.end();)
        if (!(*it)->valid())
        {
            events.on_removal(**it);
            it = m_elements.erase(it);
        }
        else
            ++it;
}

void constraint_manager2D::delegate_collisions(const std::vector<collision2D> *collisions)
{
    m_collisions = collisions;
}

void constraint_manager2D::update_contacts()
{
    KIT_PERF_FUNCTION()
    const auto ctrres = world.collisions.resolution<constraint_driven_resolution2D>();
    for (const collision2D &collision : *m_collisions)
        if (collision.collided)
            for (std::size_t i = 0; i < collision.manifold.size; i++)
            {
                const kit::commutative_tuple<kit::uuid, kit::uuid, std::size_t> hash{collision.collider1->id,
                                                                                     collision.collider2->id, i};
                const auto old_contact = m_contacts.find(hash);
                if (old_contact != m_contacts.end())
                    old_contact->second.update(&collision, ctrres->slop);
                else
                    m_contacts.emplace(hash, contact_constraint2D(world, &collision, i, ctrres->slop));
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

void constraint_manager2D::solve()
{
    KIT_ASSERT_ERROR(baumgarte_coef >= 0.f, "Baumgarte coef must be non-negative: {0}", baumgarte_coef)
    KIT_ASSERT_ERROR(baumgarte_threshold >= 0.f, "Baumgarte threshold must be non-negative: {0}", baumgarte_threshold)
    KIT_PERF_SCOPE("Constraints solve")
    if (m_elements.empty() && !m_collisions)
        return;

    if (m_collisions)
        update_contacts();

    if (warmup)
    {
        KIT_PERF_SCOPE("Warmup")
        for (const auto &ctr : m_elements)
            ctr->warmup();
        if (m_collisions)
            for (auto &[hash, contact] : m_contacts)
                contact.warmup();
    }

    {
        KIT_PERF_SCOPE("Solve")
        for (std::size_t i = 0; i < iterations; i++)
        {
            for (const auto &ctr : m_elements)
                ctr->solve();
            if (m_collisions)
                for (auto &[hash, contact] : m_contacts)
                    contact.solve();
        }
    }
    m_collisions = nullptr;
}

} // namespace ppx