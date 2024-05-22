#pragma once

#include "ppx/joints/joint2D.hpp"
#include "ppx/collision/collision2D.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"
#include "ppx/actuators/actuator2D.hpp"

namespace ppx
{
class contact2D;

template <typename T>
concept IContact2D = IJoint2D<T> && kit::DerivedFrom<T, contact2D>;

template <typename T>
concept ContactConstraint2D = IContact2D<T> && IPVConstraint2D<T>; // Use IVCConstraint2D instead of IPVConstraint2D?

template <typename T>
concept ContactActuator2D = IContact2D<T> && IActuator2D<T>;

template <typename T>
concept Contact2D = ContactConstraint2D<T> || ContactActuator2D<T>;

class contact2D : virtual public joint2D
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
} // namespace ppx