#pragma once

#include "ppx/internal/worldref.hpp"
#include "ppx/common/specs2D.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/interface/toggleable.hpp"

namespace ppx
{
class joint2D;

template <typename T>
concept Joint2D = requires() {
    requires kit::DerivedFrom<T, joint2D>;
    typename T::specs;
    requires kit::DerivedFrom<typename T::specs, specs::joint2D>;
};

class body2D;
class joint2D : public kit::indexable, public kit::toggleable, public worldref2D
{
  public:
    virtual ~joint2D() = default;

    bool bodies_collide;

    const body2D *body1() const;
    const body2D *body2() const;

    body2D *body1();
    body2D *body2();

    bool awake() const;

    const glm::vec2 &lanchor1() const;
    const glm::vec2 &lanchor2() const;

    glm::vec2 ganchor1() const;
    glm::vec2 ganchor2() const;

    bool contains(const body2D *body) const;

    virtual void solve() = 0;

  protected:
    joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2);
    joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor);
    joint2D(world2D &world, const specs::joint2D &spc);

    joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
            bool bodies_collide = true);
    joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor, bool bodies_collide = true);
    joint2D(world2D &world, body2D *body1, body2D *body2, bool bodies_collide = true);

    body2D *m_body1;
    body2D *m_body2;

    glm::vec2 m_lanchor1;
    glm::vec2 m_lanchor2;

  protected:
    bool m_no_anchors; // joint is not anchored (e.g. rotor or motor)

  private:
    void add_to_bodies();
    void remove_from_bodies();

    template <Joint2D T> friend class joint_container2D;
};

} // namespace ppx