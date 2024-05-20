#pragma once

#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class contact2D
{
  public:
    virtual ~contact2D() = default;

    bool recently_updated = true;

    const collision2D *collision() const;
    std::size_t manifold_index() const;
    bool is_new() const;

  protected:
    contact2D(const collision2D *collision, std::size_t manifold_index);

    virtual void update(const collision2D *collision, std::size_t manifold_index);

    const collision2D *m_collision;
    std::size_t m_manifold_index;
    bool m_is_new = true;
};
} // namespace ppx