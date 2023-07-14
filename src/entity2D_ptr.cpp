#include "ppx/internal/pch.hpp"
#include "ppx/entity2D_ptr.hpp"

namespace ppx
{
const_entity2D_ptr::const_entity2D_ptr(const std::vector<entity2D> *buffer, const std::size_t index)
    : m_buffer(buffer), m_index(index)
{
    if (buffer)
        m_id = (*buffer)[index].id();
}
const_entity2D_ptr::const_entity2D_ptr(const std::vector<entity2D> *buffer, std::size_t index, uuid id)
    : m_buffer(buffer), m_index(index), m_id(id)
{
}

std::size_t const_entity2D_ptr::index() const
{
    return m_index;
}
uuid const_entity2D_ptr::id() const
{
    return m_id;
}

const entity2D *const_entity2D_ptr::raw() const
{
    return &((*m_buffer)[m_index]);
}
const entity2D *const_entity2D_ptr::operator->() const
{
    return &((*m_buffer)[m_index]);
}
const entity2D &const_entity2D_ptr::operator*() const
{
    return (*m_buffer)[m_index];
}

bool const_entity2D_ptr::valid() const
{
    return m_buffer != nullptr && m_index < m_buffer->size() && m_id == (*m_buffer)[m_index].id();
}
bool const_entity2D_ptr::validate()
{
    if (valid())
        return true;
    for (const entity2D &e : *m_buffer)
        if (e.id() == m_id)
        {
            DBG_INFO("Entity pointer (index: {0}, id: {1}) had to be validated to index {2}", m_index, m_id, e.index())
            m_index = e.index();
            return true;
        }
    return false;
}

const_entity2D_ptr::operator bool() const
{
    return valid();
}

bool operator==(const const_entity2D_ptr &e1, const const_entity2D_ptr &e2)
{
    return e1.id() == e2.id();
}
bool operator!=(const const_entity2D_ptr &e1, const const_entity2D_ptr &e2)
{
    return e1.id() != e2.id();
}

entity2D_ptr::entity2D_ptr(std::vector<entity2D> *buffer, const std::size_t index) : m_buffer(buffer), m_index(index)
{
    if (buffer)
        m_id = (*buffer)[index].id();
}

std::size_t entity2D_ptr::index() const
{
    return m_index;
}
uuid entity2D_ptr::id() const
{
    return m_id;
}

entity2D *entity2D_ptr::raw() const
{
    return &((*m_buffer)[m_index]);
}
entity2D *entity2D_ptr::operator->() const
{
    return &((*m_buffer)[m_index]);
}
entity2D &entity2D_ptr::operator*() const
{
    return (*m_buffer)[m_index];
}

bool entity2D_ptr::valid() const
{
    return m_buffer != nullptr && m_index < m_buffer->size() && m_id == (*m_buffer)[m_index].id();
}
bool entity2D_ptr::validate()
{
    if (valid())
        return true;
    for (const entity2D &e : *m_buffer)
        if (e.id() == m_id)
        {
            DBG_INFO("Entity pointer (index: {0}, id: {1}) had to be validated to index {2}", m_index, m_id, e.index())
            m_index = e.index();
            return true;
        }
    return false;
}

entity2D_ptr::operator bool() const
{
    return valid();
}
entity2D_ptr::operator const_entity2D_ptr() const
{
    return const_entity2D_ptr(m_buffer, m_index, m_id);
}

bool operator==(const entity2D_ptr &e1, const entity2D_ptr &e2)
{
    return e1.id() == e2.id();
}
bool operator!=(const entity2D_ptr &e1, const entity2D_ptr &e2)
{
    return e1.id() != e2.id();
}
} // namespace ppx

namespace std
{
size_t hash<ppx::const_entity2D_ptr>::operator()(const ppx::const_entity2D_ptr &key) const
{
    return hash<ppx::uuid>()(key.id());
}

size_t hash<ppx::entity2D_ptr>::operator()(const ppx::entity2D_ptr &key) const
{
    return hash<ppx::uuid>()(key.id());
}
} // namespace std