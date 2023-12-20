#pragma once

#include "ppx/body2D.hpp"

namespace ppx
{
struct collision2D
{
    static inline constexpr std::size_t MANIFOLD_SIZE = 4;
    static inline constexpr float MIN_CONTACT_DIST = 0.1f;

    body2D::ptr current;
    body2D::ptr incoming;

    glm::vec2 normal{0.f};
    std::array<glm::vec2, MANIFOLD_SIZE> manifold;
    std::uint8_t size = 1;

    bool valid = true;

    bool add_contact_point(const glm::vec2 &contact);
    collision2D reciprocal() const;

    const glm::vec2 &touch1(std::size_t manifold_index) const;
    glm::vec2 touch2(std::size_t manifold_index) const;
};

struct contact_point_query
{
    static inline constexpr std::uint8_t MAX_LIFETIME = 8;

    contact_point_query(const glm::vec2 &first_contact);

    std::array<glm::vec2, collision2D::MANIFOLD_SIZE - 1> contacts;
    std::uint8_t size = 1;
    std::uint8_t lifetime = MAX_LIFETIME;
    std::uint8_t next = 1;

    void add_contact_point(const glm::vec2 &contact);
};
} // namespace ppx
