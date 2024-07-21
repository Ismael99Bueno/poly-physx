#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/behaviour_manager2D.hpp"

namespace ppx
{
bool behaviour_manager2D::remove(std::size_t index)
{
    if (index >= m_elements.size())
        return false;

    events.on_removal(*m_elements[index]);
    m_elements.erase(m_elements.begin() + index);

    return true;
}

template <> behaviour2D *behaviour_manager2D::from_name(const std::string &name) const
{
    for (const auto &bhv : m_elements)
        if (name == bhv->id())
            return bhv.get();
    return nullptr;
}

void behaviour_manager2D::on_body_removal_validation(body2D *body)
{
    for (const auto &bhv : m_elements)
        bhv->remove(body);
}

void behaviour_manager2D::load_forces(std::vector<state2D> &states)
{
    KIT_PERF_SCOPE("ppx::behaviour_manager2D::load_forces")
    for (const auto &bhv : m_elements)
        if (bhv->enabled())
            bhv->load_forces(states);
}

void behaviour_manager2D::process_addition(kit::scope<behaviour2D> &&bhv)
{
#ifdef DEBUG
    for (const auto &old : m_elements)
    {
        KIT_ASSERT_ERROR(*old != *bhv,
                         "Cannot add a behaviour with a name that already exists. Behaviour names act as identifiers")
    }
#endif

    m_elements.push_back(std::move(bhv));
    events.on_addition(m_elements.back().get());
}
} // namespace ppx