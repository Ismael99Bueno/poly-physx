#ifndef PPX_COLLISION_HPP
#define PPX_COLLISION_HPP

#include "ppx/body2D.hpp"

namespace ppx
{
struct collision2D
{
    body2D::ptr current;
    body2D::ptr incoming;

    glm::vec2 normal{0.f};
    std::array<glm::vec2, 2> manifold{glm::vec2(0.f), glm::vec2(0.f)};
    std::uint8_t size = 1;

    bool valid = true;

    const glm::vec2 &touch1(std::size_t manifold_index) const;
    glm::vec2 touch2(std::size_t manifold_index) const;
};
} // namespace ppx

#endif