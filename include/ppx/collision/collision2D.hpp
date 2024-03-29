#pragma once

#include "ppx/collider/collider2D.hpp"

namespace ppx
{
using manifold2D = kit::dynarray<glm::vec2, 2>;
struct collision2D
{
    collider2D *collider1 = nullptr;
    collider2D *collider2 = nullptr;

    float restitution;
    float friction;

    glm::vec2 mtv{0.f};
    manifold2D manifold;

    bool collided = false;
    mutable bool enabled = true;
};
} // namespace ppx
