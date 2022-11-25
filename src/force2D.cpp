#include "force2D.hpp"

namespace phys
{
    force2D::force2D(const std::size_t allocations) { m_entities.reserve(allocations); }

    void force2D::include(const const_entity_ptr &e) { m_entities.insert(e); }
    void force2D::exclude(const const_entity_ptr &e) { m_entities.erase(e); }
    bool force2D::contains(const const_entity_ptr &e) const { return m_entities.find(e) != m_entities.end(); }

    const std::unordered_set<const_entity_ptr> &force2D::entities() const { return m_entities; }
}