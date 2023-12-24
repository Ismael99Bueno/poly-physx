#pragma once

#include "ppx/body2D.hpp"
#include "ppx/collision/manifolds/manifold2D.hpp"

namespace ppx
{
struct collision2D
{
    body2D::ptr body1;
    body2D::ptr body2;

    glm::vec2 mtv{0.f};
    manifold2D manifold;

    bool valid = true;

    const glm::vec2 &touch1(std::size_t manifold_index) const;
    glm::vec2 touch2(std::size_t manifold_index) const;
};
} // namespace ppx
