#pragma once

#include "ppx/internal/worldref.hpp"
#include "kit/interface/identifiable.hpp"
#include <vector>

namespace ppx
{
class world2D;
template <kit::Identifiable T> class manager2D
{
  public:
    using id_type = typename T::id_type;

    manager2D(world2D &world);
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

    const T &operator[](std::size_t index) const;
    T &operator[](std::size_t index);

    const T *operator[](const id_type &id) const;
    T *operator[](const id_type &id);

    virtual bool remove(std::size_t index) = 0;
    bool remove(const T &element);
    bool remove(const id_type &id);

    std::size_t size() const;
    bool empty() const;

  protected:
    std::vector<T> m_elements;
};
} // namespace ppx