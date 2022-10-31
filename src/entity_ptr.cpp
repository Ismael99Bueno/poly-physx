#include "entity_ptr.hpp"

namespace physics
{
    entity_ptr::entity_ptr(std::vector<entity2D> &vec,
                           const std::size_t index) : m_vec(&vec), m_index(index) {}

    const entity2D *entity_ptr::operator->() const { return &((*m_vec)[m_index]); }
    const entity2D &entity_ptr::operator*() const { return (*m_vec)[m_index]; }

    entity2D *entity_ptr::operator->() { return &((*m_vec)[m_index]); }
    entity2D &entity_ptr::operator*() { return (*m_vec)[m_index]; }

    explicit entity_ptr::operator bool() const { return m_vec; }

    bool operator==(const entity_ptr &e1, const entity_ptr &e2) { return &(*e1) == &(*e2); }
    bool operator!=(const entity_ptr &e1, const entity_ptr &e2) { return !(e1 == e2); }
}