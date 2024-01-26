#include "ppx/internal/pch.hpp"
#include "ppx/manager2D.hpp"

namespace ppx
{
template <kit::Identifiable T> manager2D<T>::manager2D(world2D &world) : worldref2D(world)
{
}

template <kit::Identifiable T> const T &manager2D<T>::operator[](std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    return m_elements[index];
}

template <kit::Identifiable T> T &manager2D<T>::operator[](std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    return m_elements[index];
}

template <kit::Identifiable T> const T *manager2D<T>::operator[](const id_type &id) const
{
    for (const T &element : m_elements)
        if (element.id == id)
            return &element;
    return nullptr;
}
template <kit::Identifiable T> T *manager2D<T>::operator[](const id_type &id)
{
    for (T &element : m_elements)
        if (element.id == id)
            return &element;
    return nullptr;
}

template <kit::Identifiable T> bool manager2D<T>::remove(const T &element)
{
    return remove(element.id);
}
template <kit::Identifiable T> bool manager2D<T>::remove(const id_type &id)
{
    for (std::size_t i = 0; i < m_elements.size(); ++i)
        if (m_elements[i].id == id)
            return remove(i);
    return false;
}

template <kit::Identifiable T> std::size_t manager2D<T>::size() const
{
    return m_elements.size();
}
template <kit::Identifiable T> bool manager2D<T>::empty() const
{
    return m_elements.empty();
}

} // namespace ppx
