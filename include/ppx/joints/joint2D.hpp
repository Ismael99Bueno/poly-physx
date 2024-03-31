#pragma once

#include "ppx/body/body2D.hpp"
#include "kit/interface/indexable.hpp"

namespace ppx
{
class joint2D : public kit::indexable, public worldref2D
{
  public:
    virtual ~joint2D() = default;

    bool bodies_collide;

    body2D *body1() const;
    body2D *body2() const;

    const glm::vec2 &lanchor1() const;
    const glm::vec2 &lanchor2() const;

    glm::vec2 ganchor1() const;
    glm::vec2 ganchor2() const;

    bool contains(const body2D *body) const;

    virtual void solve() = 0;

  protected:
    joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2);
    joint2D(world2D &world, const specs::joint2D &spc, const glm::vec2 &ganchor);

    joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
            bool bodies_collide = true);
    joint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor, bool bodies_collide = true);

    body2D *m_body1;
    body2D *m_body2;

    glm::vec2 m_lanchor1;
    glm::vec2 m_lanchor2;
};
} // namespace ppx