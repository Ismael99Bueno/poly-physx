#pragma once

#include "ppx/entities/collider2D.hpp"
#include "ppx/collision/manifold/manifold2D.hpp"

namespace ppx
{
struct collision2D
{
    collider2D *collider1 = nullptr;
    collider2D *collider2 = nullptr;

    float restitution;
    float friction;

    glm::vec2 mtv{0.f};
    manifold2D manifold;

    bool collided = false;

    const glm::vec2 &touch1(std::size_t manifold_index) const;
    glm::vec2 touch2(std::size_t manifold_index) const;
};
} // namespace ppx
