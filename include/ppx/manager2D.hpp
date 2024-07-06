#pragma once

#include "ppx/internal/worldref.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/memory/ptr/scope.hpp"
#include "kit/events/event.hpp"
#include <vector>

namespace ppx
{
class world2D;

template <typename T> struct type_wrapper
{
    using value_t = T;
    using element_t = T *;
    static inline value_t *ptr(const element_t element)
    {
        return element;
    }
};
template <typename T> struct type_wrapper<kit::scope<T>>
{
    using value_t = T;
    using element_t = kit::scope<T>;
    static inline value_t *ptr(const element_t &element)
    {
        return element.get();
    }
};

template <typename T> struct manager_events
{
    using value_t = typename type_wrapper<T>::value_t;
    kit::event<value_t *> on_addition;
    kit::event<value_t &> on_removal;
};

template <typename T> class manager2D : public worldref2D, kit::non_copyable
{
  public:
    using value_t = typename type_wrapper<T>::value_t;
    using element_t = typename type_wrapper<T>::element_t;

    virtual ~manager2D() = default;

    manager_events<T> events;

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

    const value_t *at(std::size_t index) const
    {
        KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                         m_elements.size())
        return type_wrapper<T>::ptr(m_elements[index]);
    }
    value_t *at(std::size_t index)
    {
        KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                         m_elements.size())
        return type_wrapper<T>::ptr(m_elements[index]);
    }

    const value_t *operator[](std::size_t index) const
    {
        return at(index);
    }
    value_t *operator[](std::size_t index)
    {
        return at(index);
    }

    virtual bool remove(std::size_t index) = 0;
    virtual bool remove(value_t *element)
    {
        for (std::size_t i = 0; i < m_elements.size(); i++)
            if (element == type_wrapper<T>::ptr(m_elements[i]))
                return remove(i);
        return false;
    }
    bool contains(const value_t *element) const
    {
        for (const element_t &e : m_elements)
            if (element == type_wrapper<T>::ptr(e))
                return true;
        return false;
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
    std::vector<element_t> m_elements;

    friend class world2D;
};

template <typename T>
concept Identifiable = kit::Identifiable<typename type_wrapper<T>::value_t>;

template <Identifiable T> class idmanager2D : public manager2D<T>
{
  public:
    using value_t = typename type_wrapper<T>::value_t;
    using element_t = typename type_wrapper<T>::element_t;
    using id_t = typename value_t::id_type;

    const value_t *at(const id_t &id) const
    {
        for (const element_t &element : this->m_elements)
            if (element->id() == id)
                return type_wrapper<T>::ptr(element);
        return nullptr;
    }
    value_t *at(const id_t &id)
    {
        for (const element_t &element : this->m_elements)
            if (element->id() == id)
                return type_wrapper<T>::ptr(element);
        return nullptr;
    }

    const value_t *operator[](const id_t &id) const
    {
        return at(id);
    }
    value_t *operator[](const id_t &id)
    {
        return at(id);
    }

    using manager2D<T>::remove;
    bool remove(const id_t &id)
    {
        for (std::size_t i = 0; i < this->m_elements.size(); i++)
            if (this->m_elements[i]->id() == id)
                return remove(i);
        return false;
    }

    bool contains(const id_t &id) const
    {
        return (*this)[id] != nullptr;
    }

  protected:
    using manager2D<T>::manager2D;
};

} // namespace ppx