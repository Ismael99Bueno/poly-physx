#include "ppx/internal/pch.hpp"
#include "ppx/manager2D.hpp"
#include "ppx/entities/body2D.hpp"
#include "ppx/entities/collider2D.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/joints/spring2D.hpp"

namespace ppx
{
template <Identifiable T> manager2D<T>::manager2D(world2D &world) : worldref2D(world)
{
}

template <Identifiable T> const typename manager2D<T>::value_type &manager2D<T>::operator[](std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())

    if constexpr (is_scope)
        return *m_elements[index];
    else
        return m_elements[index];
}

template <Identifiable T> typename manager2D<T>::value_type &manager2D<T>::operator[](std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    if constexpr (is_scope)
        return *m_elements[index];
    else
        return m_elements[index];
}

template <Identifiable T> const typename manager2D<T>::value_type *manager2D<T>::operator[](const id_type &id) const
{
    if constexpr (is_scope)
    {
        for (const T &element : m_elements)
            if (element->id == id)
                return element.get();
    }
    else
        for (const T &element : m_elements)
            if (element.id == id)
                return &element;
    return nullptr;
}
template <Identifiable T> typename manager2D<T>::value_type *manager2D<T>::operator[](const id_type &id)
{
    if constexpr (is_scope)
    {
        for (T &element : m_elements)
            if (element->id == id)
                return element.get();
    }
    else
        for (T &element : m_elements)
            if (element.id == id)
                return &element;
    return nullptr;
}

template <Identifiable T> bool manager2D<T>::remove(const value_type &element)
{
    if constexpr (kit::Indexable<T>)
        return remove(element.index);
    else
        return remove(element.id);
}
template <Identifiable T> bool manager2D<T>::remove(const id_type &id)
{
    if constexpr (is_scope)
    {
        for (std::size_t i = 0; i < m_elements.size(); ++i)
            if (m_elements[i]->id == id)
                return remove(i);
    }
    else
        for (std::size_t i = 0; i < m_elements.size(); ++i)
            if (m_elements[i].id == id)
                return remove(i);
    return false;
}

template <Identifiable T> bool manager2D<T>::contains(const value_type &element) const
{
    return contains(element.id);
}
template <Identifiable T> bool manager2D<T>::contains(const id_type &id) const
{
    return (*this)[id] != nullptr;
}

template <Identifiable T> std::size_t manager2D<T>::size() const
{
    return m_elements.size();
}
template <Identifiable T> bool manager2D<T>::empty() const
{
    return m_elements.empty();
}
template <Identifiable T> void manager2D<T>::clear()
{
    for (std::size_t i = m_elements.size() - 1; i < m_elements.size(); --i)
        remove(i);
}

template class manager2D<body2D>;
template class manager2D<collider2D>;
template class manager2D<spring2D>;
template class manager2D<kit::scope<behaviour2D>>;
template class manager2D<kit::scope<constraint2D>>;

} // namespace ppx
