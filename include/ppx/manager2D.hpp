#pragma once

#include "ppx/internal/worldref.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/scope.hpp"
#include <vector>

namespace ppx
{
class world2D;

template <typename T> struct type_wrapper
{
    using value = T;
    static inline constexpr bool is_scope = false;
};
template <typename T> struct type_wrapper<kit::scope<T>>
{
    using value = T;
    static inline constexpr bool is_scope = true;
};

template <typename T>
concept Identifiable = kit::Identifiable<typename type_wrapper<T>::value>;

template <Identifiable T>
class manager2D

    : kit::non_copyable,
      public worldref2D
{
  public:
    using value_type = typename type_wrapper<T>::value;
    using id_type = typename value_type::id_type;
    static inline constexpr bool is_scope = type_wrapper<T>::is_scope;

    virtual ~manager2D() = default;

    auto begin() const
    {
        return m_elements.begin();
    }
    auto end() const
    {
        return m_elements.end();
    }

    auto begin()
    {
        return m_elements.begin();
    }
    auto end()
    {
        return m_elements.end();
    }

    const value_type &operator[](std::size_t index) const
    {
        KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                         m_elements.size())

        if constexpr (is_scope)
            return *m_elements[index];
        else
            return m_elements[index];
    }
    value_type &operator[](std::size_t index)
    {
        KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                         m_elements.size())

        if constexpr (is_scope)
            return *m_elements[index];
        else
            return m_elements[index];
    }

    const value_type *operator[](const id_type &id) const
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
    value_type *operator[](const id_type &id)
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

    virtual bool remove(std::size_t index) = 0;
    bool remove(const value_type &element)
    {
        if constexpr (kit::Indexable<T>)
            return remove(element.index);
        else
            return remove(element.id);
    }
    bool remove(const id_type &id)
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

    bool contains(const value_type &element) const
    {
        return contains(element.id);
    }
    bool contains(const id_type &id) const
    {
        return (*this)[id] != nullptr;
    }

    std::size_t size() const
    {
        return m_elements.size();
    }
    bool empty() const
    {
        return m_elements.empty();
    }
    void clear()
    {
        for (std::size_t i = m_elements.size() - 1; i < m_elements.size(); --i)
            remove(i);
    }

  protected:
    manager2D(world2D &world) : worldref2D(world)
    {
    }
    std::vector<T> m_elements;
};
} // namespace ppx