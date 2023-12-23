#pragma once

#include "ppx/body2D.hpp"

namespace ppx
{
struct collision2D
{
    static inline constexpr std::size_t MANIFOLD_SIZE = 4;
    static inline constexpr float MIN_CONTACT_DIST = 0.1f;

    body2D::ptr body1;
    body2D::ptr body2;

    glm::vec2 normal{0.f};
    std::array<glm::vec2, MANIFOLD_SIZE> manifold;
    std::uint8_t size = 1;

    bool valid = true;

    bool add_contact_point(const glm::vec2 &contact);

    const glm::vec2 &touch1(std::size_t manifold_index) const;
    glm::vec2 touch2(std::size_t manifold_index) const;
};
} // namespace ppx
