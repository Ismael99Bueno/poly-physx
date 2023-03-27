#include "entity2D_ptr.hpp"
#include "debug.hpp"

namespace ppx
{
    const_entity2D_ptr::const_entity2D_ptr(const std::vector<entity2D> *buffer,
                                           const std::size_t index) : m_buffer(buffer),
                                                                      m_index(index)
    {
        if (buffer)
            m_id = (*buffer)[index].id();
    }

    std::size_t const_entity2D_ptr::index() const { return m_index; }
    std::size_t const_entity2D_ptr::id() const { return m_id; }

    const entity2D *const_entity2D_ptr::raw() const { return &((*m_buffer)[m_index]); }
    const entity2D *const_entity2D_ptr::operator->() const { return &((*m_buffer)[m_index]); }
    const entity2D &const_entity2D_ptr::operator*() const { return (*m_buffer)[m_index]; }

    bool const_entity2D_ptr::is_valid() const { return m_index < m_buffer->size() && m_id == (*m_buffer)[m_index].id(); }
    bool const_entity2D_ptr::try_validate()
    {
        if (is_valid())
            return true;
        for (const entity2D &e : *m_buffer)
            if (e.id() == m_id)
            {
                DBG_LOG("Validating pointer, from index %zu and id %zu to index %zu and id %zu.\n", m_index, m_id, e.index(), e.id())
                m_id = e.id();
                m_index = e.index();
                return true;
            }
        DBG_LOG("Failed to validate pointer with index %zu and id %zu (THIS IS OK).\n", m_index, m_id)
        return false;
    }

    const_entity2D_ptr::operator bool() const { return m_buffer; }

    bool operator==(const const_entity2D_ptr &e1, const const_entity2D_ptr &e2) { return e1.id() == e2.id(); }
    bool operator!=(const const_entity2D_ptr &e1, const const_entity2D_ptr &e2) { return e1.id() != e2.id(); }

    entity2D_ptr::entity2D_ptr(std::vector<entity2D> *buffer,
                               const std::size_t index) : m_buffer(buffer),
                                                          m_index(index)
    {
        if (buffer)
            m_id = (*buffer)[index].id();
    }

    std::size_t entity2D_ptr::index() const { return m_index; }
    std::size_t entity2D_ptr::id() const { return m_id; }

    entity2D *entity2D_ptr::raw() const { return &((*m_buffer)[m_index]); }
    entity2D *entity2D_ptr::operator->() const { return &((*m_buffer)[m_index]); }
    entity2D &entity2D_ptr::operator*() const { return (*m_buffer)[m_index]; }

    bool entity2D_ptr::is_valid() const { return m_index < m_buffer->size() && m_id == (*m_buffer)[m_index].id(); }
    bool entity2D_ptr::try_validate()
    {
        if (is_valid())
            return true;
        for (const entity2D &e : *m_buffer)
            if (e.id() == m_id)
            {
                DBG_LOG("Validating pointer, from index %zu and id %zu to index %zu and id %zu.\n", m_index, m_id, e.index(), e.id())
                m_id = e.id();
                m_index = e.index();
                return true;
            }
        DBG_LOG("Failed to validate pointer with index %zu and id %zu (THIS IS OK).\n", m_index, m_id)
        return false;
    }

    entity2D_ptr::operator bool() const { return m_buffer; }
    entity2D_ptr::operator const_entity2D_ptr() const { return const_entity2D_ptr(m_buffer, m_index); }

    bool operator==(const entity2D_ptr &e1, const entity2D_ptr &e2) { return e1.id() == e2.id(); }
    bool operator!=(const entity2D_ptr &e1, const entity2D_ptr &e2) { return e1.id() != e2.id(); }
}

namespace std
{
    size_t hash<ppx::const_entity2D_ptr>::operator()(const ppx::const_entity2D_ptr &key) const
    {
        return hash<std::size_t>()(key.id());
    }

    size_t hash<ppx::entity2D_ptr>::operator()(const ppx::entity2D_ptr &key) const
    {
        return hash<std::size_t>()(key.id());
    }
}