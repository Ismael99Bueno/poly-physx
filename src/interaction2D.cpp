#include "interaction2D.hpp"

namespace physics
{
    interaction2D::interaction2D(const std::size_t allocations) { m_entities.reserve(allocations); }

    void interaction2D::add(const entity_ptr &e)
    {
        m_entities.insert(e);
        e->add(*this);
    }

    void interaction2D::remove(const entity_ptr &e)
    {
        m_entities.erase(e);
        e->remove(*this);
    }

    bool interaction2D::contains(const const_entity_ptr &e) const { return m_entities.find(e) != m_entities.end(); }
}