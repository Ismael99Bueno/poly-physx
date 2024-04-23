#pragma once

#include "ppx/common/shapes2D.hpp"
#include "ppx/collision/filter.hpp"
#include "kit/container/dynarray.hpp"

namespace ppx
{
class body2D;
class collider2D;
class distance_joint2D;
class revolute_joint2D;
class weld_joint2D;
class joint2D;
class rotor_joint2D;
class motor_joint2D;
class spring2D;
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
    struct properties
    {
        float density = 1.f;
        float charge_density = 1.f;
        float restitution = 0.f;
        float friction = 0.8f;
        kit::dynarray<glm::vec2, PPX_MAX_VERTICES> vertices = polygon::square(5.f);
        float radius = 2.5f;
        stype shape = stype::POLYGON;
        filter collision_filter{};
    } props;
    static collider2D from_instance(const ppx::collider2D &collider);
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
    struct properties
    {
        float mass = 1.f;
        float charge = 1.f;
        std::vector<collider2D> colliders{};
        btype type = btype::DYNAMIC;
    } props;
    static body2D from_instance(const ppx::body2D &body);
};

struct joint2D
{
    std::size_t bindex1 = SIZE_MAX;
    std::size_t bindex2 = SIZE_MAX;

    body2D bspecs1{};
    body2D bspecs2{};

    bool bodies_collide = true;
};

struct rotor_joint2D : joint2D
{
    struct properties
    {
        float torque = 0.f;
        float correction_factor = 0.05f;
        float target_speed = 0.f;
        float min_offset = 0.f;
        float max_offset = 0.f;
        bool spin_indefinitely = false;
    } props;
    static rotor_joint2D from_instance(const ppx::rotor_joint2D &rotj);
};

struct motor_joint2D : joint2D
{
    struct properties
    {
        float force = 0.f;
        float correction_factor = 0.05f;
        float target_speed = 0.f;
        glm::vec2 target_offset{0.f};
    } props;
    static motor_joint2D from_instance(const ppx::motor_joint2D &motj);
};

struct distance_joint2D : joint2D
{
    glm::vec2 ganchor1{FLT_MAX};
    glm::vec2 ganchor2{FLT_MAX};
    bool deduce_distance = true;
    struct properties
    {
        float min_distance = 0.f;
        float max_distance = 0.f;
    } props;
    static distance_joint2D from_instance(const ppx::distance_joint2D &dj);
};

struct revolute_joint2D : joint2D
{
    glm::vec2 ganchor{FLT_MAX};
    static revolute_joint2D from_instance(const ppx::revolute_joint2D &revj);
};

struct weld_joint2D : joint2D
{
    glm::vec2 ganchor{FLT_MAX};
    static weld_joint2D from_instance(const ppx::weld_joint2D &weldj);
};

struct spring2D : joint2D
{
    glm::vec2 ganchor1{FLT_MAX};
    glm::vec2 ganchor2{FLT_MAX};
    bool deduce_length = false;
    struct properties
    {
        float frequency = 1.f;
        float damping_ratio = 0.2f;
        float length = 0.f;

        std::uint32_t non_linear_terms = 0;
        float non_linear_contribution = 0.001f;
    } props;

    static spring2D from_instance(const ppx::spring2D &sp);
};

struct contraption2D
{
    std::vector<body2D> bodies{};
    std::vector<distance_joint2D> distance_joints{};
    std::vector<spring2D> springs{};

    static contraption2D rope(const glm::vec2 &start, const glm::vec2 &end, std::uint32_t segments,
                              float spring_anchor_spacing, const body2D::properties &node_props = {},
                              const spring2D::properties &spring_props = {}, bool fixed_start = true,
                              bool fixed_end = true);
    static contraption2D chain(const glm::vec2 &start, const glm::vec2 &end, std::uint32_t segments,
                               float spring_anchor_spacing, const body2D::properties &node_props = {},
                               const distance_joint2D::properties &dj_props = {}, bool fixed_start = true,
                               bool fixed_end = true);

    static contraption2D soft_body(const std::vector<glm::vec2> &anchors, const body2D::properties &body_props = {},
                                   const spring2D::properties &spring_props = {});
    static contraption2D soft_body(float radius, std::uint32_t segments, const body2D::properties &body_props = {},
                                   const spring2D::properties &spring_props = {});

    template <std::random_access_iterator AnchorIt>
    static contraption2D soft_body(AnchorIt it1, AnchorIt it2, const body2D::properties &body_props = {},
                                   const spring2D::properties &spring_props = {})
    {
        const std::size_t size = std::distance(it1, it2);
        KIT_ASSERT_ERROR(size > 1, "Soft body must have at least 2 anchors");

        contraption2D contraption;
        contraption.springs.reserve(size * (size - 1) / 2);

        for (std::size_t i = 0; i < size; ++i)
            for (std::size_t j = i + 1; j < size; ++j)
            {
                auto p1 = it1 + i;
                auto p2 = it1 + j;
                const body2D spc1 = {.position = *p1, .props = body_props};
                const body2D spc2 = {.position = *p2, .props = body_props};
                contraption.springs.push_back({{.bspecs1 = spc1, .bspecs2 = spc2}, *p1, *p2, false, spring_props});
            }
        return contraption;
    }
};

} // namespace ppx::specs