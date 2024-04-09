#pragma once

#include "ppx/collider/collider2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
using manifold2D = kit::dynarray<geo::contact_point2D, 2>;
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
