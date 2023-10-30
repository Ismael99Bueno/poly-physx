#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
constraint_manager2D::constraint_manager2D(world2D &world, const std::size_t allocations) : m_world(world)
{
    m_constraints.reserve(allocations);
}

bool constraint_manager2D::remove_constraint(std::size_t index, const kit::event<const constraint2D &> &event_callback)
{
    if (index >= m_constraints.size())
    {
        KIT_WARN("Constraint index exceeds array bounds. Aborting... - index: {0}, size: {1}", index,
                 m_constraints.size())
        return false;
    }
    event_callback(*m_constraints[index]);
    m_constraints.erase(m_constraints.begin() + (long)index);
    return true;
}
bool constraint_manager2D::remove_constraint(const constraint2D *ctr,
                                             const kit::event<const constraint2D &> &event_callback)
{
    for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it)
        if (it->get() == ctr)
        {
            event_callback(*ctr);
            m_constraints.erase(it);
            return true;
        }
    return false;
}
bool constraint_manager2D::remove_constraint(kit::uuid id, const kit::event<const constraint2D &> &event_callback)
{
    for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it)
        if ((*it)->id == id)
        {
            event_callback(**it);
            m_constraints.erase(it);
            return true;
        }
    return false;
}

void constraint_manager2D::clear_constraints(const kit::event<const constraint2D &> &event_callback)
{
    for (const auto &ctr : m_constraints)
        event_callback(*ctr);
    m_constraints.clear();
}

void constraint_manager2D::validate(const kit::event<const constraint2D &> &event_callback)
{
    for (auto it = m_constraints.begin(); it != m_constraints.end();)
        if (!(*it)->valid())
        {
            event_callback(**it);
            it = m_constraints.erase(it);
        }
        else
            ++it;
}

void constraint_manager2D::solve_constraints() const
{
    KIT_PERF_FUNCTION()
    if (m_constraints.empty())
        return;
    for (const auto &ctr : m_constraints)
        ctr->warmup();

    const std::size_t iters = 10;
    for (std::size_t i = 0; i < iters; i++)
        for (const auto &ctr : m_constraints)
            ctr->solve();
}

const std::vector<kit::scope<constraint2D>> &constraint_manager2D::constraints() const
{
    return m_constraints;
}

} // namespace ppx