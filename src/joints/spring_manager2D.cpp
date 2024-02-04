#include "ppx/internal/pch.hpp"
#include "ppx/joints/spring_manager2D.hpp"

namespace ppx
{
void spring_manager2D::process_addition(spring2D &sp)
{
    sp.index = m_elements.size() - 1;
    KIT_ASSERT_ERROR(sp.joint.valid(), "The spring joint must be valid to be able to add it into the simulation")
    events.on_addition(sp);
}

spring2D::const_ptr spring_manager2D::ptr(const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    return {&m_elements, index};
}
spring2D::ptr spring_manager2D::ptr(const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    return {&m_elements, index};
}

std::vector<spring2D::const_ptr> spring_manager2D::from_ids(const kit::uuid id1, const kit::uuid id2) const
{
    std::vector<spring2D::const_ptr> springs;
    springs.reserve(m_elements.size());

    for (const spring2D &sp : m_elements)
        if ((sp.joint.body1().id == id1 && sp.joint.body2().id == id2) ||
            (sp.joint.body1().id == id2 && sp.joint.body2().id == id1))
            springs.emplace_back(&m_elements, sp.index);
    return springs;
}

std::vector<spring2D::ptr> spring_manager2D::from_ids(const kit::uuid id1, const kit::uuid id2)
{
    std::vector<spring2D::ptr> springs;
    springs.reserve(m_elements.size());

    for (const spring2D &sp : m_elements)
        if ((sp.joint.body1().id == id1 && sp.joint.body2().id == id2) ||
            (sp.joint.body1().id == id2 && sp.joint.body2().id == id1))
            springs.emplace_back(&m_elements, sp.index);
    return springs;
}

bool spring_manager2D::remove(const std::size_t index)
{
    if (index >= m_elements.size())
        return false;

    events.on_early_removal(m_elements[index]);
    if (index != m_elements.size() - 1)
    {
        m_elements[index] = m_elements.back();
        m_elements[index].index = index;
    }
    m_elements.pop_back();
    events.on_late_removal(index);
    return true;
}

void spring_manager2D::apply_forces()
{
    KIT_PERF_FUNCTION()
    for (spring2D &sp : m_elements)
        sp.apply_force_to_bodies();
}

void spring_manager2D::validate()
{
    std::size_t index = 0;
    for (auto it = m_elements.begin(); it != m_elements.end(); index++)
        if (!it->joint.valid())
        {
            events.on_early_removal(*it);
            it = m_elements.erase(it);
            events.on_late_removal(index);
        }
        else
        {
            it->index = index;
            ++it;
        }
}
} // namespace ppx