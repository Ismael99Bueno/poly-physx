#include "ppx/internal/pch.hpp"
#include "ppx/joints/spring_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
spring_manager2D::spring_manager2D(world2D &world) : worldref2D(world)
{
}

void spring_manager2D::process_addition(spring2D &sp)
{
    sp.index = m_springs.size() - 1;
    KIT_ASSERT_ERROR(sp.joint.valid(), "The spring joint must be valid to be able to add it into the simulation")
    world.events.on_spring_addition(sp);
}

const spring2D &spring_manager2D::operator[](const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_springs.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_springs.size())
    return m_springs[index];
}
spring2D &spring_manager2D::operator[](const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_springs.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_springs.size())
    return m_springs[index];
}

spring2D::const_ptr spring_manager2D::ptr(const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_springs.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_springs.size())
    return {&m_springs, index};
}
spring2D::ptr spring_manager2D::ptr(const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_springs.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_springs.size())
    return {&m_springs, index};
}

const spring2D *spring_manager2D::operator[](const kit::uuid id) const
{
    for (const spring2D &sp : m_springs)
        if (sp.id == id)
            return &sp;
    return nullptr;
}
spring2D *spring_manager2D::operator[](const kit::uuid id)
{
    for (spring2D &sp : m_springs)
        if (sp.id == id)
            return &sp;
    return nullptr;
}

std::vector<spring2D::const_ptr> spring_manager2D::from_ids(const kit::uuid id1, const kit::uuid id2) const
{
    std::vector<spring2D::const_ptr> springs;
    springs.reserve(m_springs.size());

    for (const spring2D &sp : m_springs)
        if ((sp.joint.body1().id == id1 && sp.joint.body2().id == id2) ||
            (sp.joint.body1().id == id2 && sp.joint.body2().id == id1))
            springs.emplace_back(&m_springs, sp.index);
    return springs;
}

std::vector<spring2D::ptr> spring_manager2D::from_ids(const kit::uuid id1, const kit::uuid id2)
{
    std::vector<spring2D::ptr> springs;
    springs.reserve(m_springs.size());

    for (const spring2D &sp : m_springs)
        if ((sp.joint.body1().id == id1 && sp.joint.body2().id == id2) ||
            (sp.joint.body1().id == id2 && sp.joint.body2().id == id1))
            springs.emplace_back(&m_springs, sp.index);
    return springs;
}

bool spring_manager2D::remove(const std::size_t index)
{
    if (index >= m_springs.size())
    {
        KIT_WARN("Spring index exceeds array bounds. Aborting... - index: {0}, size: {1}", index, m_springs.size())
        return false;
    }
    world.events.on_early_spring_removal(m_springs[index]);
    if (index != m_springs.size() - 1)
    {
        m_springs[index] = m_springs.back();
        m_springs[index].index = index;
    }
    m_springs.pop_back();
    world.events.on_late_spring_removal(index);
    return true;
}
bool spring_manager2D::remove(const spring2D &sp)
{
    return remove(sp.index);
}
bool spring_manager2D::remove(kit::uuid id)
{
    for (const spring2D &sp : m_springs)
        if (sp.id == id)
            return remove(sp.index);
    return false;
}

std::size_t spring_manager2D::size() const
{
    return m_springs.size();
}

void spring_manager2D::clear()
{
    for (std::size_t i = m_springs.size() - 1; i < m_springs.size(); i--)
        remove(i);
}

void spring_manager2D::apply_forces()
{
    KIT_PERF_FUNCTION()
    for (spring2D &sp : m_springs)
        sp.apply_force_to_bodies();
}

void spring_manager2D::validate()
{
    std::size_t index = 0;
    for (auto it = m_springs.begin(); it != m_springs.end(); index++)
        if (!it->joint.valid())
        {
            world.events.on_early_spring_removal(*it);
            it = m_springs.erase(it);
            world.events.on_late_spring_removal(index);
        }
        else
        {
            it->index = index;
            ++it;
        }
}
} // namespace ppx