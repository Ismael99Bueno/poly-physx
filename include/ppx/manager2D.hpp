#pragma once

#include "ppx/internal/worldref.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/memory/scope.hpp"
#include "kit/interface/non_copyable.hpp"
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

    const value_type &operator[](std::size_t index) const;
    value_type &operator[](std::size_t index);

    const value_type *operator[](const id_type &id) const;
    value_type *operator[](const id_type &id);

    virtual bool remove(std::size_t index) = 0;
    bool remove(const value_type &element);
    bool remove(const id_type &id);

    bool contains(const value_type &element) const;
    bool contains(const id_type &id) const;

    std::size_t size() const;
    bool empty() const;
    void clear();

  protected:
    manager2D(world2D &world);
    std::vector<T> m_elements;
};
} // namespace ppx