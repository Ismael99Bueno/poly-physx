#include "ppx/internal/pch.hpp"
#include "ppx/collision/spring_solver2D.hpp"
#include "kit/profile/perf.hpp"

#include "kit/utility/multithreading.hpp"
#include "kit/utility/utils.hpp"

#if defined(PPX_MULTITHREADED) && defined(KIT_PROFILE)
#pragma message(                                                                                                       \
        "Multithreading for PPX will be disabled because the thread unsafe profiling features of cpp-kit are enabled")
#undef PPX_MULTITHREADED
#endif

namespace ppx
{
static float map_zero_one_zero_inf(const float x, const float slope)
{
    return slope * logf((1 + x) / (1 - x));
}
static float map_one_zero_zero_inf(const float x, const float slope)
{
    return slope * logf((2 - x) / x);
}

float spring_solver2D::s_rigidity = map_zero_one_zero_inf(spring_solver2D::s_rigidity_coeff, 400.f);
float spring_solver2D::s_restitution = map_zero_one_zero_inf(spring_solver2D::s_restitution_coeff, 2.f);
float spring_solver2D::s_friction = map_zero_one_zero_inf(spring_solver2D::s_friction_coeff, 2.f);

float spring_solver2D::rigidity_coeff()
{
    return s_rigidity_coeff;
}
float spring_solver2D::restitution_coeff()
{
    return s_restitution_coeff;
}
float spring_solver2D::friction_coeff()
{
    return s_friction_coeff;
}

void spring_solver2D::rigidity_coeff(const float rigidity)
{
    KIT_ASSERT_ERROR(rigidity >= 0.f && rigidity < 1.f, "Rigidity must lie between [0, 1)")
    s_rigidity_coeff = rigidity;
    s_rigidity = map_zero_one_zero_inf(rigidity, 200.f);
}
void spring_solver2D::restitution_coeff(const float restitution)
{
    KIT_ASSERT_ERROR(restitution > 0.f && restitution <= 1.f, "Restitution must lie between (0, 1]")
    s_restitution_coeff = restitution;
    s_restitution = map_one_zero_zero_inf(restitution, 2.f);
}
void spring_solver2D::friction_coeff(const float friction)
{
    KIT_ASSERT_ERROR(friction >= 0.f && friction < 1.f, "Friction must lie between [0, 1]")
    s_friction_coeff = friction;
    s_friction = map_zero_one_zero_inf(friction, 2.f);
}

float spring_solver2D::rigidity()
{
    return s_rigidity;
}
float spring_solver2D::restitution()
{
    return s_restitution;
}
float spring_solver2D::friction()
{
    return s_friction;
}

void spring_solver2D::solve(const std::vector<collision2D> &collisions) const
{
    KIT_PERF_FUNCTION()
#ifdef PPX_MULTITHREADED
    kit::const_for_each_mt<PPX_THREAD_COUNT, collision2D>(
        collisions, [this](const std::size_t thread_index, const collision2D &colis) {
            if (colis.valid)
                solve_and_apply_collision_forces(colis);
        });
#else
    for (const collision2D &colis : collisions)
        if (colis.valid)
            solve_and_apply_collision_forces(colis);
#endif
}

void spring_solver2D::solve_and_apply_collision_forces(const collision2D &colis) const
{
    KIT_PERF_FUNCTION()
    const glm::vec2 rel1 = colis.touch1 - colis.current->position();
    const glm::vec2 rel2 = colis.touch2 - colis.incoming->position();

    const glm::vec2 relvel = (colis.incoming->velocity_at(rel2) - colis.current->velocity_at(rel1));
    const glm::vec2 reltouch = colis.touch2 - colis.touch1;

    const glm::vec2 normal_dir = glm::normalize(colis.normal);

    const glm::vec2 orth_vel = glm::dot(normal_dir, relvel) * normal_dir;
    const glm::vec2 tangent_vel = relvel - orth_vel;

    const glm::vec2 force = reltouch * s_rigidity + orth_vel * s_restitution + tangent_vel * s_friction;

    const float torque1 = kit::cross2D(rel1, force), torque2 = kit::cross2D(force, rel2);
    colis.current->apply_simulation_force(force);
    colis.current->apply_simulation_torque(torque1);

    colis.incoming->apply_simulation_force(-force);
    colis.incoming->apply_simulation_torque(torque2);
}
} // namespace ppx