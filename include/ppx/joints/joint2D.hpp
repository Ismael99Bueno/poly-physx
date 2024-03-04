#pragma once

#include "ppx/entities/specs2D.hpp"
#include "ppx/entities/body2D.hpp"

namespace ppx
{
class joint2D : public worldref2D
{
  public:
    using specs = specs::joint2D;

    const body2D::ptr &body1() const;
    const body2D::ptr &body2() const;

    const glm::vec2 &lanchor1() const;
    const glm::vec2 &lanchor2() const;

    bool valid() const;

  protected:
    joint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &ganchor1,
            const glm::vec2 &ganchor2);
    joint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &ganchor);

    body2D::ptr m_body1;
    body2D::ptr m_body2;

    glm::vec2 m_lanchor1;
    glm::vec2 m_lanchor2;
};
} // namespace ppx