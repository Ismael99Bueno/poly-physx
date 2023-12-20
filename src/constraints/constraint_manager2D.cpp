#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
constraint_manager2D::constraint_manager2D(world2D &world) : m_world(world), m_events(world.events)
{
}

bool constraint_manager2D::remove(std::size_t index)
{
    if (index >= m_constraints.size())
    {
        KIT_WARN("Constraint index exceeds array bounds. Aborting... - index: {0}, size: {1}", index,
                 m_constraints.size())
        return false;
    }
    m_world.events.on_constraint_removal(*m_constraints[index]);
    m_constraints.erase(m_constraints.begin() + (long)index);
    return true;
}
bool constraint_manager2D::remove(const constraint2D *ctr)
{
    for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it)
        if (it->get() == ctr)
        {
            m_world.events.on_constraint_removal(*ctr);
            m_constraints.erase(it);
            return true;
        }
    return false;
}

const constraint2D &constraint_manager2D::operator[](std::size_t index) const
{
    return *m_constraints[index];
}
constraint2D &constraint_manager2D::operator[](std::size_t index)
{
    return *m_constraints[index];
}

static std::optional<std::size_t> index_from_id(const kit::uuid id, const std::vector<kit::scope<constraint2D>> &vec)
{
    for (std::size_t i = 0; i < vec.size(); i++)
        if (vec[i]->id == id)
            return i;
    return {};
}

const constraint2D *constraint_manager2D::operator[](const kit::uuid id) const
{
    const auto index = index_from_id(id, m_constraints);
    return index ? m_constraints[index.value()].get() : nullptr;
}
constraint2D *constraint_manager2D::operator[](const kit::uuid id)
{
    const auto index = index_from_id(id, m_constraints);
    return index ? m_constraints[index.value()].get() : nullptr;
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
    return from_ids<const constraint2D *>(ids, m_constraints);
}
std::vector<constraint2D *> constraint_manager2D::operator[](const std::vector<kit::uuid> &ids)
{
    return from_ids<constraint2D *>(ids, m_constraints);
}

bool constraint_manager2D::remove(kit::uuid id)
{
    for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it)
        if ((*it)->id == id)
        {
            m_world.events.on_constraint_removal(**it);
            m_constraints.erase(it);
            return true;
        }
    return false;
}

void constraint_manager2D::clear()
{
    for (const auto &ctr : m_constraints)
        m_world.events.on_constraint_removal(*ctr);
    m_constraints.clear();
}

void constraint_manager2D::validate()
{
    for (auto it = m_constraints.begin(); it != m_constraints.end();)
        if (!(*it)->valid())
        {
            m_world.events.on_constraint_removal(**it);
            it = m_constraints.erase(it);
        }
        else
        {
            (*it)->world = &m_world;
            ++it;
        }
}

std::size_t constraint_manager2D::size() const
{
    return m_constraints.size();
}

void constraint_manager2D::delegate_collisions(const std::vector<collision2D> *collisions)
{
    m_collisions = collisions;
}

void constraint_manager2D::solve()
{
    KIT_PERF_FUNCTION()
    if (m_constraints.empty() && !m_collisions)
        return;

    if (m_collisions)
    {
        m_contacts.clear();
        for (const collision2D &collision : *m_collisions)
            for (std::size_t i = 0; i < collision.size; i++)
                m_contacts.emplace_back(collision, i).world = &m_world;
    }

    if (warmup)
    {
        for (const auto &ctr : m_constraints)
            ctr->warmup();
        if (m_collisions)
            for (contact_constraint2D &contact : m_contacts)
                contact.warmup();
    }

    for (std::size_t i = 0; i < iterations; i++)
    {
        for (const auto &ctr : m_constraints)
            ctr->solve();
        if (m_collisions)
            for (contact_constraint2D &contact : m_contacts)
                contact.solve();
    }
    m_collisions = nullptr;
}

} // namespace ppx