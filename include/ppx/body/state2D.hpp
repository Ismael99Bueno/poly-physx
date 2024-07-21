#pragma once

#include "ppx/common/alias.hpp"
#include "ppx/common/specs2D.hpp"

namespace ppx
{
struct state2D
{
    state2D(const transform2D &centroid = {}, const glm::vec2 &velocity = glm::vec2(0.f), float angular_velocity = 0.f);

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

    glm::mat3 tmatrix() const;
    glm::mat3 inv_tmatrix() const;

    float inv_mass() const;
    float inv_inertia() const;

    bool is_dynamic() const;
    bool is_kinematic() const;
    bool is_static() const;

    transform2D centroid;
    glm::vec2 lposition;

    glm::vec2 velocity;
    float angular_velocity;

    float mass;
    float imass;

    float inertia;
    float iinertia;

    glm::vec2 force;
    float torque;

    specs::body2D::btype type;
    float charge;
    glm::vec2 charge_centroid;
};
} // namespace ppx