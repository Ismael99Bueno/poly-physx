#include "interaction2D.hpp"

namespace physics
{
    interaction2D::interaction2D(const std::size_t allocations) { m_entities.reserve(allocations); }

    void interaction2D::include(const entity_ptr &e)
    {
        m_entities.insert(e);
        e->include(*this);
    }

    void interaction2D::exclude(const entity_ptr &e)
    {
        m_entities.erase(e);
        e->exclude(*this);
    }

    bool interaction2D::contains(const const_entity_ptr &e) const { return m_entities.find(e) != m_entities.end(); }

    const std::unordered_set<const_entity_ptr> &interaction2D::entities() const { return m_entities; }
}