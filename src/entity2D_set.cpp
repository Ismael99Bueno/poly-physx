#include "entity2D_set.hpp"
#include "debug.hpp"

namespace phys
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
    void entity2D_set::exclude(const const_entity2D_ptr &e)
    {
        m_entities.erase(std::remove(m_entities.begin(), m_entities.end(), e), m_entities.end());
    }
    bool entity2D_set::contains(const const_entity2D_ptr &e) const { return std::find(m_entities.begin(), m_entities.end(), e) != m_entities.end(); }
    void entity2D_set::clear() { m_entities.clear(); }
    std::size_t entity2D_set::size() const { return m_entities.size(); }

    const std::vector<const_entity2D_ptr> &entity2D_set::entities() const { return m_entities; }
}