#pragma once

#include "ppx/entities/body2D.hpp"
#include "ppx/collision/manifold/manifold2D.hpp"

namespace ppx
{
struct collision2D
{
    body2D *body1 = nullptr;
    body2D *body2 = nullptr;

    glm::vec2 mtv{0.f};
    manifold2D manifold;

    bool collided = false;

    const glm::vec2 &touch1(std::size_t manifold_index) const;
    glm::vec2 touch2(std::size_t manifold_index) const;
};
} // namespace ppx
