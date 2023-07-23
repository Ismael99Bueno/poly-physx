#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/engine2D.hpp"

namespace ppx
{
behaviour2D::behaviour2D(const std::string &name, const std::size_t allocations) : kit::identifiable<std::string>(name)
{
    m_included.reserve(allocations);
}

void behaviour2D::validate()
{
    for (auto it = m_included.begin(); it != m_included.end();)
        if (!(*it))
            it = m_included.erase(it);
        else
            ++it;
}

void behaviour2D::include(const body2D::const_ptr &bd)
{
    m_included.push_back(bd);
}
void behaviour2D::exclude(const body2D &bd)
{
    for (auto it = m_included.begin(); it != m_included.end(); ++it)
        if (*(*it) == bd)
        {
            m_included.erase(it);
            break;
        }
}
bool behaviour2D::contains(const body2D &bd) const
{
    for (const body2D::const_ptr &bdptr : m_included)
        if (*bdptr == bd)
            return true;
    return false;
}
float behaviour2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const auto &bd : m_included)
        ke += bd->kinetic_energy();
    return ke;
}

float behaviour2D::energy(const body2D &bd) const
{
    return bd.kinetic_energy() + potential_energy(bd);
}
float behaviour2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

void behaviour2D::clear()
{
    m_included.clear();
}
std::size_t behaviour2D::size() const
{
    return m_included.size();
}

const std::vector<body2D::const_ptr> &behaviour2D::entities() const
{
    return m_included;
}

#ifdef KIT_USE_YAML_CPP
YAML::Node behaviour2D::encode() const
{
    YAML::Node node;
    for (const auto &bd : m_included)
        node["Entities"].push_back(bd->index());
    node["Entities"].SetStyle(YAML::EmitterStyle::Flow);
    return node;
}
bool behaviour2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() < 1)
        return false;
    clear();
    for (const YAML::Node &n : node["Entities"])
        include((*m_parent)[n.as<std::size_t>()]);
    return true;
}
#endif
} // namespace ppx