#include "ppx/internal/pch.hpp"
#include "ppx/joints/spring_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
spring_manager2D::spring_manager2D(world2D &world) : m_world(world)
{
}

spring2D::ptr spring_manager2D::process_addition(spring2D &sp)
{
    sp.world = &m_world;
    sp.index = m_springs.size() - 1;
    sp.world = &m_world;

    const spring2D::ptr sp_ptr = {&m_springs, sp.index};

    KIT_ASSERT_ERROR(sp.joint.valid(), "The spring joint must be valid to be able to add it into the simulation")
    m_world.events.on_spring_addition(sp_ptr);
    return sp_ptr;
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

static std::optional<std::size_t> index_from_id(const kit::uuid id, const std::vector<spring2D> &vec)
{
    for (std::size_t i = 0; i < vec.size(); i++)
        if (vec[i].id == id)
            return i;
    return {};
}

spring2D::const_ptr spring_manager2D::operator[](const kit::uuid id) const
{
    const auto index = index_from_id(id, m_springs);
    return index ? ptr(index.value()) : nullptr;
}
spring2D::ptr spring_manager2D::operator[](const kit::uuid id)
{
    const auto index = index_from_id(id, m_springs);
    return index ? ptr(index.value()) : nullptr;
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

bool spring_manager2D::remove(std::size_t index)
{
    if (index >= m_springs.size())
    {
        KIT_WARN("Spring index exceeds array bounds. Aborting... - index: {0}, size: {1}", index, m_springs.size())
        return false;
    }
    m_world.events.on_early_spring_removal(m_springs[index]);
    if (index != m_springs.size() - 1)
    {
        m_springs[index] = m_springs.back();
        m_springs[index].index = index;
    }
    m_springs.pop_back();
    m_world.events.on_late_spring_removal(std::move(index));
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
            m_world.events.on_early_spring_removal(*it);
            it = m_springs.erase(it);
            m_world.events.on_late_spring_removal(std::move(index));
        }
        else
        {
            it->world = &m_world;
            it->index = index;
            ++it;
        }
}
} // namespace ppx