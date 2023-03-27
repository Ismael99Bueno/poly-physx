#include "entity2D_set.hpp"
#include "debug.hpp"

namespace ppx
{
    entity2D_set::entity2D_set(const std::size_t allocations) { m_entities.reserve(allocations); }

    void entity2D_set::validate()
    {
        for (auto it = m_entities.begin(); it != m_entities.end();)
            if (!it->try_validate())
                it = m_entities.erase(it);
            else
                ++it;
    }

    void entity2D_set::include(const const_entity2D_ptr &e) { m_entities.emplace_back(e); }
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
}