#include "force2D.hpp"

namespace physics
{
    force2D::force2D(const std::size_t allocations) { m_entities.reserve(allocations); }

    void force2D::add(const entity_ptr &e)
    {
        m_entities.insert(e);
        e->add(*this);
    }

    void force2D::remove(const entity_ptr &e)
    {
        m_entities.erase(e);
        e->remove(*this);
    }

    bool force2D::contains(const const_entity_ptr &e) const { return m_entities.find(e) != m_entities.end(); }
}