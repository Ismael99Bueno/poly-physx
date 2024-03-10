#pragma once

#include "ppx/entities/shapes2D.hpp"
#include "kit/container/dynarray.hpp"

namespace ppx
{
class body2D;
class collider2D;
class distance_joint2D;
class joint2D;
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
        float restitution = 0.2f;
        float friction = 0.8f;
        kit::dynarray<glm::vec2, PPX_MAX_VERTICES> vertices = polygon::square(5.f);
        float radius = 2.5f;
        stype shape = stype::POLYGON;
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
};

struct distance_joint2D : joint2D
{
    glm::vec2 ganchor1{FLT_MAX};
    glm::vec2 ganchor2{FLT_MAX};
    static distance_joint2D from_instance(const ppx::distance_joint2D &dj);
};

struct spring2D : joint2D
{
    glm::vec2 ganchor1{FLT_MAX};
    glm::vec2 ganchor2{FLT_MAX};
    struct properties
    {
        float stiffness = 1.f;
        float damping = 0.f;
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
                               bool fixed_start = true, bool fixed_end = true);

    static contraption2D soft_body(const std::vector<glm::vec2> &anchors, const body2D::properties &body_props = {},
                                   const spring2D::properties &spring_props = {});
    static contraption2D soft_body(float radius, std::uint32_t segments, const body2D::properties &body_props = {},
                                   const spring2D::properties &spring_props = {});

    template <std::input_iterator AnchorIt>
    static contraption2D soft_body(AnchorIt it1, AnchorIt it2, const body2D::properties &body_props = {},
                                   const spring2D::properties &spring_props = {})
    {
        const std::size_t size = std::distance(it1, it2);
        KIT_ASSERT_ERROR(size > 1, "Soft body must have at least 2 anchors");
        contraption2D contraption;
        contraption.bodies.reserve(size);
        contraption.springs.reserve(size * (size - 1) / 2);
        for (auto it = it1; it != it2; ++it)
            contraption.bodies.push_back(body2D{.position = *it, .props = body_props});
        for (std::size_t i = 0; i < size; ++i)
            for (std::size_t j = i + 1; j < size; ++j)
                contraption.springs.push_back(
                    {{i, j}, contraption.bodies[i].position, contraption.bodies[j].position, spring_props});
        return contraption;
    }
};

} // namespace ppx::specs