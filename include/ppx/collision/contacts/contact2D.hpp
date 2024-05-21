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

    const collider2D *collider1() const;
    const collider2D *collider2() const;

    collider2D *collider1();
    collider2D *collider2();

    std::size_t manifold_index() const;
    bool is_new() const;

  protected:
    contact2D(const collision2D *collision, std::size_t manifold_index);

    virtual void update(const collision2D *collision, std::size_t manifold_index);

    const collision2D *m_collision;
    collider2D *m_collider1;
    collider2D *m_collider2;

    std::size_t m_manifold_index;
    bool m_is_new = true;
};

class contact_joint2D;
class contact_constraint2D;

template <typename T>
concept ContactJoint2D = kit::DerivedFrom<T, contact_joint2D>;

template <typename T>
concept ContactConstraint2D = kit::DerivedFrom<T, contact_constraint2D>;

template <typename T>
concept Contact2D = ContactJoint2D<T> || ContactConstraint2D<T>;
} // namespace ppx