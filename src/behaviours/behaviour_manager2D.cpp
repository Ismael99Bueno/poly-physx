#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/behaviour_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
behaviour_manager2D::behaviour_manager2D(world2D &world) : worldref2D(world), m_events(world.events)
{
}

const behaviour2D &behaviour_manager2D::operator[](std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_behaviours.size(), "Index exceeds container size: {0}", index)
    return *m_behaviours[index];
}
behaviour2D &behaviour_manager2D::operator[](std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_behaviours.size(), "Index exceeds container size: {0}", index)
    return *m_behaviours[index];
}

const behaviour2D *behaviour_manager2D::operator[](const std::string &name) const
{
    return from_name<behaviour2D>(name);
}
behaviour2D *behaviour_manager2D::operator[](const std::string &name)
{
    return from_name<behaviour2D>(name);
}

bool behaviour_manager2D::remove(std::size_t index)
{
    if (index >= m_behaviours.size())
    {
        KIT_WARN("Behaviour index exceeds array bounds. Aborting... - index: {0}, size: {1}", index,
                 m_behaviours.size())
        return false;
    }
    world.events.on_behaviour_removal(*m_behaviours[index]);
    m_behaviours.erase(m_behaviours.begin() + index);
    return true;
}
bool behaviour_manager2D::remove(const behaviour2D *bhv)
{
    for (auto it = m_behaviours.begin(); it != m_behaviours.end(); ++it)
        if (it->get() == bhv)
        {
            world.events.on_behaviour_removal(*bhv);
            m_behaviours.erase(it);
            return true;
        }
    return false;
}
bool behaviour_manager2D::remove(const std::string &name)
{
    for (auto it = m_behaviours.begin(); it != m_behaviours.end(); ++it)
        if ((*it)->id == name)
        {
            world.events.on_behaviour_removal(**it);
            m_behaviours.erase(it);
            return true;
        }
    return false;
}

template <> behaviour2D *behaviour_manager2D::from_name(const std::string &name) const
{
    for (const auto &bhv : m_behaviours)
        if (name == bhv->id)
            return bhv.get();
    return nullptr;
}

void behaviour_manager2D::clear()
{
    for (const auto &bhv : m_behaviours)
        world.events.on_behaviour_removal(*bhv);
    m_behaviours.clear();
}

void behaviour_manager2D::validate()
{
    for (const auto &bhv : m_behaviours)
        bhv->validate();
}

std::size_t behaviour_manager2D::size() const
{
    return m_behaviours.size();
}

void behaviour_manager2D::apply_forces()
{
    KIT_PERF_FUNCTION()
    for (const auto &bhv : m_behaviours)
        if (bhv->enabled)
            bhv->apply_force_to_bodies();
}
} // namespace ppx