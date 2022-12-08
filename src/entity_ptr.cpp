#include "entity_ptr.hpp"
#include "debug.h"

namespace phys
{
    const_entity_ptr::const_entity_ptr(const std::vector<entity2D> *buffer,
                                       const std::size_t index) : m_buffer(buffer),
                                                                  m_index(index),
                                                                  m_id((*buffer)[index].id()) {}

    std::size_t const_entity_ptr::index() const { return m_index; }
    const entity2D *const_entity_ptr::raw() const { return &((*m_buffer)[m_index]); }
    const entity2D *const_entity_ptr::operator->() const { return &((*m_buffer)[m_index]); }
    const entity2D &const_entity_ptr::operator*() const { return (*m_buffer)[m_index]; }

    bool const_entity_ptr::is_valid() const { return m_id == (*m_buffer)[m_index].id(); }
    bool const_entity_ptr::try_validate()
    {
        if (is_valid())
            return true;
        for (const entity2D &e : *m_buffer)
            if (e.id() == m_id)
            {
                DBG_LOG("Validating pointer, from index %zu and id %llu to index %zu and id %llu.\n", m_index, m_id, e.index(), e.id())
                m_id = e.id();
                m_index = e.index();
                return true;
            }
        DBG_LOG("Failed to validate pointer with index %zu and id %llu.\n", m_index, m_id)
        return false;
    }

    const_entity_ptr::operator bool() const { return m_buffer; }

    bool operator==(const const_entity_ptr &e1, const const_entity_ptr &e2) { return &(*e1) == &(*e2); }
    bool operator!=(const const_entity_ptr &e1, const const_entity_ptr &e2) { return !(e1 == e2); }

    entity_ptr::entity_ptr(std::vector<entity2D> *buffer,
                           const std::size_t index) : m_buffer(buffer),
                                                      m_index(index),
                                                      m_id((*buffer)[index].id()) {}

    std::size_t entity_ptr::index() const { return m_index; }
    entity2D *entity_ptr::raw() const { return &((*m_buffer)[m_index]); }
    entity2D *entity_ptr::operator->() const { return &((*m_buffer)[m_index]); }
    entity2D &entity_ptr::operator*() const { return (*m_buffer)[m_index]; }

    bool entity_ptr::is_valid() const { return m_id == (*m_buffer)[m_index].id(); }
    bool entity_ptr::validate()
    {
        if (is_valid())
            return true;
        for (const entity2D &e : *m_buffer)
            if (e.id() == m_id)
            {
                DBG_LOG("Validating pointer, from index %zu and id %llu to index %zu and id %llu.\n", m_index, m_id, e.index(), e.id())
                m_id = e.id();
                m_index = e.index();
                return true;
            }
        DBG_LOG("Failed to validate pointer with index %zu and id %llu.\n", m_index, m_id)
        return false;
    }

    entity_ptr::operator bool() const { return m_buffer; }
    entity_ptr::operator const_entity_ptr() const { return const_entity_ptr(m_buffer, m_index); }

    bool operator==(const entity_ptr &e1, const entity_ptr &e2) { return &(*e1) == &(*e2); }
    bool operator!=(const entity_ptr &e1, const entity_ptr &e2) { return !(e1 == e2); }
}

namespace std
{
    size_t hash<phys::const_entity_ptr>::operator()(const phys::const_entity_ptr &key) const
    {
        return hash<std::size_t>()(key.m_index);
    }

    size_t hash<phys::entity_ptr>::operator()(const phys::entity_ptr &key) const
    {
        return hash<std::size_t>()(key.m_index);
    }
}