#include "entity_ptr.hpp"

namespace physics
{
    const_entity_ptr::const_entity_ptr(const std::vector<entity2D> &buffer,
                                       const std::size_t index) : m_buffer(&buffer), m_index(index) {}

    const entity2D *const_entity_ptr::operator->() const { return &((*m_buffer)[m_index]); }
    const entity2D &const_entity_ptr::operator*() const { return (*m_buffer)[m_index]; }

    const_entity_ptr::operator bool() const { return m_buffer; }

    bool operator==(const const_entity_ptr &e1, const const_entity_ptr &e2) { return &(*e1) == &(*e2); }
    bool operator!=(const const_entity_ptr &e1, const const_entity_ptr &e2) { return !(e1 == e2); }

    entity_ptr::entity_ptr(std::vector<entity2D> &buffer,
                           const std::size_t index) : m_buffer(&buffer), m_index(index) {}

    entity2D *entity_ptr::operator->() const { return &((*m_buffer)[m_index]); }
    entity2D &entity_ptr::operator*() const { return (*m_buffer)[m_index]; }

    entity_ptr::operator bool() const { return m_buffer; }
    entity_ptr::operator const_entity_ptr() const { return const_entity_ptr(*m_buffer, m_index); }

    bool operator==(const entity_ptr &e1, const entity_ptr &e2) { return &(*e1) == &(*e2); }
    bool operator!=(const entity_ptr &e1, const entity_ptr &e2) { return !(e1 == e2); }
}

namespace std
{
    size_t hash<physics::const_entity_ptr>::operator()(const physics::const_entity_ptr &key) const
    {
        return hash<std::size_t>()(key.m_index);
    }

    size_t hash<physics::entity_ptr>::operator()(const physics::entity_ptr &key) const
    {
        return hash<std::size_t>()(key.m_index);
    }
}