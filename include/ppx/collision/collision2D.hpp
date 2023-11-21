#ifndef PPX_COLLISION_HPP
#define PPX_COLLISION_HPP

#include "ppx/body2D.hpp"

namespace ppx
{
struct collision2D
{
    body2D::ptr current;
    body2D::ptr incoming;

    glm::vec2 touch1{0.f};
    glm::vec2 touch2{0.f};
    glm::vec2 normal{0.f};

    bool valid = true;
};
} // namespace ppx

#endif