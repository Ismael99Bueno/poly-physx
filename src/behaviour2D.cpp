#include "ppx/pch.hpp"
#include "ppx/behaviour2D.hpp"
#include "debug/debug.hpp"

namespace ppx
{
    behaviour2D::behaviour2D(const char *name,
                               const std::size_t allocations) : m_name(name) { m_entities.reserve(allocations); }

    void behaviour2D::validate()
    {
        for (auto it = m_entities.begin(); it != m_entities.end();)
            if (!it->try_validate())
                it = m_entities.erase(it);
            else
                ++it;
    }

    void behaviour2D::include(const const_entity2D_ptr &e) { m_entities.push_back(e); }
    void behaviour2D::exclude(const entity2D &e)
    {
        for (auto it = m_entities.begin(); it != m_entities.end(); ++it)
            if (*(*it) == e)
            {
                m_entities.erase(it);
                break;
            }
    }
    bool behaviour2D::contains(const entity2D &e) const
    {
        for (const const_entity2D_ptr &entt : m_entities)
            if (*entt == e)
                return true;
        return false;
    }
    float behaviour2D::kinetic_energy() const
    {
        float ke = 0.f;
        for (const auto &e : m_entities)
            ke += e->kinetic_energy();
        return ke;
    }

    void behaviour2D::clear() { m_entities.clear(); }
    std::size_t behaviour2D::size() const { return m_entities.size(); }

    const std::vector<const_entity2D_ptr> &behaviour2D::entities() const { return m_entities; }
    const char *behaviour2D::name() const { return m_name; }

#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const behaviour2D &set)
    {
        out << YAML::BeginMap;
        set.write(out);
        out << YAML::EndMap;
        return out;
    }
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    Node convert<ppx::behaviour2D>::encode(const ppx::behaviour2D &set)
    {
        return set.encode();
    }
    bool convert<ppx::behaviour2D>::decode(const Node &node, ppx::behaviour2D &set)
    {
        return set.decode(node);
    };
}
#endif