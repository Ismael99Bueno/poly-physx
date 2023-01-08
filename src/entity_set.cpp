#include "entity_set.hpp"
#include "debug.h"

namespace phys
{
    entity_set::entity_set(const std::size_t allocations) { m_entities.reserve(allocations); }

    void entity_set::validate()
    {
        for (auto it = m_entities.begin(); it != m_entities.end();)
            if (!it->try_validate())
                it = m_entities.erase(it);
            else
                ++it;
    }

    void entity_set::include(const const_entity_ptr &e) { m_entities.emplace_back(e); }
    void entity_set::exclude(const const_entity_ptr &e)
    {
        m_entities.erase(std::remove(m_entities.begin(), m_entities.end(), e), m_entities.end());
    }
    bool entity_set::contains(const const_entity_ptr &e) const { return std::find(m_entities.begin(), m_entities.end(), e) != m_entities.end(); }
    void entity_set::clear() { m_entities.clear(); }
    std::size_t entity_set::size() const { return m_entities.size(); }

    const std::vector<const_entity_ptr> &entity_set::entities() const { return m_entities; }
}