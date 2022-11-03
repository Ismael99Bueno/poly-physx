#include "constrain2D.hpp"
#include <algorithm>

namespace physics
{
    constrain2D::constrain2D(const std::size_t allocations) { m_entities.reserve(allocations); }

    void constrain2D::add(const const_entity_ptr &e) { m_entities.push_back(e); }

    void constrain2D::remove(const const_entity_ptr &e)
    {
        m_entities.erase(std::remove(m_entities.begin(), m_entities.end(), e), m_entities.end());
    }

    bool constrain2D::contains(const const_entity_ptr &e) const
    {
        return std::find(m_entities.begin(), m_entities.end(), e) != m_entities.end();
    }
}