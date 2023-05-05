#include "ppx/pch.hpp"
#include "ppx/entity2D_set.hpp"
#include "debug/debug.hpp"

namespace ppx
{
    entity2D_set::entity2D_set(const char *name,
                               const std::size_t allocations) : m_name(name) { m_entities.reserve(allocations); }

    void entity2D_set::validate()
    {
        for (auto it = m_entities.begin(); it != m_entities.end();)
            if (!it->try_validate())
                it = m_entities.erase(it);
            else
                ++it;
    }

    void entity2D_set::include(const const_entity2D_ptr &e) { m_entities.push_back(e); }
    void entity2D_set::exclude(const entity2D &e)
    {
        for (auto it = m_entities.begin(); it != m_entities.end(); ++it)
            if (*(*it) == e)
            {
                m_entities.erase(it);
                break;
            }
    }
    bool entity2D_set::contains(const entity2D &e) const
    {
        for (const const_entity2D_ptr &entt : m_entities)
            if (*entt == e)
                return true;
        return false;
    }
    float entity2D_set::kinetic_energy() const
    {
        float ke = 0.f;
        for (const auto &e : m_entities)
            ke += e->kinetic_energy();
        return ke;
    }

    void entity2D_set::clear() { m_entities.clear(); }
    std::size_t entity2D_set::size() const { return m_entities.size(); }

    const std::vector<const_entity2D_ptr> &entity2D_set::entities() const { return m_entities; }
    const char *entity2D_set::name() const { return m_name; }

#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const entity2D_set &set)
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
    Node convert<ppx::entity2D_set>::encode(const ppx::entity2D_set &set)
    {
        return set.encode();
    }
    bool convert<ppx::entity2D_set>::decode(const Node &node, ppx::entity2D_set &set)
    {
        return set.decode(node);
    };
}
#endif