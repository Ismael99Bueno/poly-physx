#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/behaviour2D.hpp"

namespace ppx
{
behaviour2D::behaviour2D(const char *name, const std::size_t allocations) : m_name(name)
{
    m_included.reserve(allocations);
}

void behaviour2D::validate()
{
    for (auto it = m_included.begin(); it != m_included.end();)
        if (!it->validate())
            it = m_included.erase(it);
        else
            ++it;
}

void behaviour2D::include(const const_entity2D_ptr &e)
{
    m_included.push_back(e);
}
void behaviour2D::exclude(const entity2D &e)
{
    for (auto it = m_included.begin(); it != m_included.end(); ++it)
        if (*(*it) == e)
        {
            m_included.erase(it);
            break;
        }
}
bool behaviour2D::contains(const entity2D &e) const
{
    for (const const_entity2D_ptr &entt : m_included)
        if (*entt == e)
            return true;
    return false;
}
float behaviour2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const auto &e : m_included)
        ke += e->kinetic_energy();
    return ke;
}

float behaviour2D::energy(const entity2D &e) const
{
    return e.kinetic_energy() + potential_energy(e);
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

const std::vector<const_entity2D_ptr> &behaviour2D::entities() const
{
    return m_included;
}
const char *behaviour2D::name() const
{
    return m_name;
}

#ifdef YAML_CPP_COMPAT
void behaviour2D::write(YAML::Emitter &out) const
{
    out << YAML::Key << "UUID" << YAML::Value << id();
    out << YAML::Key << "Entities" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto &e : m_included)
        out << e.index();
    out << YAML::EndSeq;
}
YAML::Node behaviour2D::encode() const
{
    YAML::Node node;
    node["UUID"] = (std::uint64_t)id();
    for (const auto &e : m_included)
        node["Entities"].push_back(e.index());
    node["Entities"].SetStyle(YAML::EmitterStyle::Flow);
    return node;
}
bool behaviour2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() < 2)
        return false;
    id(node["UUID"].as<std::uint64_t>());
    clear();
    for (const YAML::Node &n : node["Entities"])
        include({m_entities, n.as<std::size_t>()});
    return true;
}
YAML::Emitter &operator<<(YAML::Emitter &out, const behaviour2D &bhv)
{
    out << YAML::BeginMap;
    bhv.write(out);
    out << YAML::EndMap;
    return out;
}
#endif
} // namespace ppx

#ifdef YAML_CPP_COMPAT
namespace YAML
{
Node convert<ppx::behaviour2D>::encode(const ppx::behaviour2D &bhv)
{
    return bhv.encode();
}
bool convert<ppx::behaviour2D>::decode(const Node &node, ppx::behaviour2D &bhv)
{
    return bhv.decode(node);
};
} // namespace YAML
#endif