#pragma once

#include "ppx/common/alias.hpp"
#include "ppx/collision/filter.hpp"
#include "kit/container/dynarray.hpp"
#include "rk/integration/integrator.hpp"

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 8
#endif

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
class spring_joint2D;
class ball_joint2D;
class prismatic_joint2D;
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
        float min_angle = 0.f;
        float max_angle = 0.f;
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

struct ball_joint2D : joint2D
{
    bool deduce_angle = true;
    struct properties
    {
        float min_angle = 0.f;
        float max_angle = 0.f;
    } props;
    static ball_joint2D from_instance(const ppx::ball_joint2D &bj);
};

struct prismatic_joint2D : joint2D
{
    glm::vec2 ganchor1{FLT_MAX};
    glm::vec2 ganchor2{FLT_MAX};
    bool deduce_axis = true;
    struct properties
    {
        glm::vec2 axis{1.f, 0.f};
    } props;
    static prismatic_joint2D from_instance(const ppx::prismatic_joint2D &pj);
};

struct spring_joint2D : joint2D
{
    glm::vec2 ganchor1{FLT_MAX};
    glm::vec2 ganchor2{FLT_MAX};
    bool deduce_length = false;
    struct properties
    {
        float frequency = 1.f;
        float damping_ratio = 0.2f;
        float min_length = 0.f;
        float max_length = 0.f;

        std::uint32_t non_linear_terms = 0;
        float non_linear_contribution = 0.001f;
    } props;

    static spring_joint2D from_instance(const ppx::spring_joint2D &sp);
};

struct joint_manager2D
{
    struct constraints2D
    {
        std::uint32_t velocity_iterations = 8;
        std::uint32_t position_iterations = 3;
        bool warmup = true;
        bool baumgarte_correction = true;

        float baumgarte_coef = 0.035f;
        float baumgarte_threshold = 0.1f;
        float slop = 0.15f;

        float max_position_correction = 0.2f;
        float position_resolution_speed = 0.2f;
    } constraints;
};

struct island_manager2D
{
    float lower_sleep_energy_threshold = 0.001f;
    float upper_sleep_energy_threshold = 0.012f;
    std::uint32_t body_count_mid_threshold_reference = 100;
    float sleep_time_threshold = 1.5f;
    bool enable_split = true;
    bool enable_sleep = true;
#ifdef KIT_PROFILE
    bool multithreaded = false;
#else
    bool multithreaded = true;
#endif
};

struct collision_manager2D
{
    struct broad2D
    {
#ifdef KIT_PROFILE
        bool multithreaded = false;
#else
        bool multithreaded = true;
#endif
        std::size_t parallel_submissions = PPX_THREAD_COUNT;
    } detection;

    struct contacts2D
    {
        float base_lifetime = 0.4f;
        float per_contact_lifetime_reduction = 0.1f;
    } contacts;
};

struct world2D
{
    struct
    {
        bool semi_implicit_integration = true;
        rk::butcher_tableau<float> tableau = rk::butcher_tableau<float>::rk1;
        rk::timestep<float> timestep{1.e-3f};
    } integrator;
    joint_manager2D joints;
    island_manager2D islands;
    collision_manager2D collision;
};

struct contraption2D
{
    std::vector<body2D> bodies{};
    std::vector<distance_joint2D> distance_joints{};
    std::vector<spring_joint2D> springs{};
    std::vector<revolute_joint2D> revolute_joints{};
    std::vector<weld_joint2D> weld_joints{};
    std::vector<rotor_joint2D> rotor_joints{};
    std::vector<motor_joint2D> motor_joints{};
    std::vector<ball_joint2D> ball_joints{};
    std::vector<prismatic_joint2D> prismatic_joints{};

    void add_offset_to_joint_indices(std::size_t offset);

    static contraption2D rope(const glm::vec2 &start, const glm::vec2 &end, std::uint32_t segments,
                              float anchor_spacing, const body2D::properties &body_props = {},
                              const spring_joint2D::properties &spring_props = {}, bool fixed_start = true,
                              bool fixed_end = true);
    static contraption2D chain(const glm::vec2 &start, const glm::vec2 &end, std::uint32_t segments,
                               float anchor_spacing, const body2D::properties &body_props = {},
                               const distance_joint2D::properties &dj_props = {}, bool fixed_start = true,
                               bool fixed_end = true);

    static contraption2D soft_body(const std::vector<glm::vec2> &anchors, const body2D::properties &body_props = {},
                                   const spring_joint2D::properties &spring_props = {});
    static contraption2D soft_body(float radius, std::uint32_t segments, const body2D::properties &body_props = {},
                                   const spring_joint2D::properties &spring_props = {});

    template <std::random_access_iterator AnchorIt>
    static contraption2D soft_body(AnchorIt it1, AnchorIt it2, const body2D::properties &body_props = {},
                                   const spring_joint2D::properties &spring_props = {})
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