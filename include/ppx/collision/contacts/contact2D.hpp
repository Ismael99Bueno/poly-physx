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

    collider2D *collider1() const;
    collider2D *collider2() const;

    const geo::contact_point2D &point() const;
    const glm::vec2 &normal() const;

    float restitution() const;
    float friction() const;

    void increment_age();
    bool recently_updated() const;
    bool expired() const;
    bool asleep() const;
    std::uint32_t life_expectancy() const;

    void on_enter();
    void on_exit();

    void on_pre_solve();
    void on_post_solve();

    bool is_contact() const override final;

  protected:
    contact2D(const collision2D *collision, std::size_t manifold_index);

    virtual void update(const collision2D *collision, std::size_t manifold_index);

    collider2D *m_collider1;
    collider2D *m_collider2;

    std::uint32_t m_age = 0;

    geo::contact_point2D m_point;
    glm::vec2 m_mtv;
    glm::vec2 m_normal;

    float m_restitution;
    float m_friction;
};
} // namespace ppx