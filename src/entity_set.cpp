#include "entity_set.hpp"
#include "debug.h"

namespace phys
{
    entity_set::entity_set(const std::size_t allocations) { m_entities.reserve(allocations); }

    bool entity_set::validate()
    {
        std::vector<const_entity_ptr> invalids;
        invalids.reserve(m_entities.size());
        for (auto it = m_entities.begin(); it != m_entities.end();)
            if (!it->is_valid())
            {
                invalids.emplace_back(*it);
                it = m_entities.erase(it);
            }
            else
                ++it;
        for (const_entity_ptr &e : invalids)
            if (e.try_validate())
                m_entities.insert(e);
        DBG_LOG_IF(invalids.empty() && !m_entities.empty(), "Validate method did not find any invalid entity pointers.\n")
        DBG_LOG_IF(!invalids.empty() && !m_entities.empty(), "Validate method found %zu invalid entity pointers.\n", invalids.size())
        return !invalids.empty();
    }

    void entity_set::include(const const_entity_ptr &e) { m_entities.insert(e); }
    void entity_set::exclude(const const_entity_ptr &e) { m_entities.erase(e); }
    bool entity_set::contains(const const_entity_ptr &e) const { return m_entities.find(e) != m_entities.end(); }
    std::size_t entity_set::size() const { return m_entities.size(); }

    const std::unordered_set<const_entity_ptr> &entity_set::entities() const { return m_entities; }
}