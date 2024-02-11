#pragma once

#include "ppx/entities/shapes2D.hpp"
#include "kit/container/dynarray.hpp"

namespace ppx
{
class body2D;
class collider2D;
class distance_joint2D;
class spring2D;
class joint_proxy2D;
} // namespace ppx

namespace ppx::specs
{
struct collider2D
{
    enum class stype
    {
        POLYGON = 0,
        CIRCLE = 1
    };
    glm::vec2 position{0.f};
    float rotation = 0.f;
    float density = 1.f;
    float charge_density = 1.f;
    float restitution = 0.5f;
    float friction = 0.5f;
    kit::dynarray<glm::vec2, PPX_MAX_VERTICES> vertices = polygon::square(5.f);
    float radius = 2.5f;
    stype shape = stype::POLYGON;
    static collider2D from_collider(const ppx::collider2D &collider);
};

struct body2D
{
    enum class btype
    {
        DYNAMIC = 0,
        KINEMATIC = 1,
        STATIC = 2
    };
    glm::vec2 position{0.f};
    glm::vec2 velocity{0.f};
    float rotation = 0.f;
    float angular_velocity = 0.f;
    float mass = 1.f;
    float charge = 1.f;
    std::vector<collider2D> colliders{};
    btype type = btype::DYNAMIC;
    static body2D from_body(const ppx::body2D &body);
};

struct joint_proxy2D
{
    std::size_t bindex1 = SIZE_MAX;
    std::size_t bindex2 = SIZE_MAX;
    glm::vec2 anchor1{0.f}, anchor2{0.f};
    static joint_proxy2D from_joint_proxy(const ppx::joint_proxy2D &jp);
};

struct distance_joint2D
{
    joint_proxy2D joint;
    static distance_joint2D from_distance_joint(const ppx::distance_joint2D &dj);
};

struct spring2D
{
    joint_proxy2D joint;
    float stiffness = 1.f;
    float damping = 0.f;
    float length = 0.f;

    std::uint32_t non_linear_terms = 0;
    float non_linear_contribution = 0.001f;

    static spring2D from_spring(const ppx::spring2D &sp);
};

} // namespace ppx::specs