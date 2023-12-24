#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/vec2.hpp>
#include <cstdint>
#include <array>

namespace ppx
{
struct manifold2D
{
    static inline constexpr std::size_t CAPACITY = 4;
    std::array<glm::vec2, CAPACITY> contacts;
    std::uint8_t size;
};

} // namespace ppx
