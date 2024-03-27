#pragma once

#include "kit/utility/transform.hpp"

namespace ppx
{
struct state2D
{
    kit::transform2D<float> centroid;
    glm::vec2 gposition;
    glm::vec2 lposition;

    glm::vec2 velocity;
    float angular_velocity;

    glm::vec2 local_centroid_point(const glm::vec2 &gpoint) const;
    glm::vec2 global_centroid_point(const glm::vec2 &lpoint) const;

    glm::vec2 local_position_point(const glm::vec2 &gpoint) const;
    glm::vec2 global_position_point(const glm::vec2 &lpoint) const;

    glm::vec2 local_vector(const glm::vec2 &gvector) const;
    glm::vec2 global_vector(const glm::vec2 &lvector) const;

    glm::vec2 lvelocity_at_from_centroid(const glm::vec2 &lpoint) const;
    glm::vec2 lvelocity_at_from_position(const glm::vec2 &lpoint) const;
    glm::vec2 gvelocity_at(const glm::vec2 &gpoint) const;

    glm::vec2 velocity_at_centroid_offset(const glm::vec2 &offset) const;
    glm::vec2 velocity_at_position_offset(const glm::vec2 &offset) const;
};
} // namespace ppx