#include "ppx/internal/pch.hpp"
#include "ppx/collision/solvers/spring_driven_solver2D.hpp"
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

float spring_driven_solver2D::rigidity()
{
    KIT_ASSERT_ERROR(rigidity_coeff >= 0.f && rigidity_coeff < 1.f, "Rigidity coefficient must lie between [0, 1)")
    return map_zero_one_zero_inf(rigidity_coeff, 200.f);
}
float spring_driven_solver2D::restitution()
{
    KIT_ASSERT_ERROR(restitution_coeff > 0.f && restitution_coeff <= 1.f,
                     "Restitution coefficient must lie between (0, 1]")
    return map_one_zero_zero_inf(restitution_coeff, 2.f);
}
float spring_driven_solver2D::friction()
{
    KIT_ASSERT_ERROR(friction_coeff >= 0.f && friction_coeff < 1.f, "Friction coefficient must lie between [0, 1)")
    return map_zero_one_zero_inf(friction_coeff, 2.f);
}

void spring_driven_solver2D::solve(const std::vector<collision2D> &collisions) const
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

std::tuple<glm::vec2, float, float> spring_driven_solver2D::compute_collision_forces(const collision2D &colis,
                                                                                     std::size_t manifold_index) const
{
    const glm::vec2 rel1 = colis.touch1(manifold_index) - colis.current->position();
    const glm::vec2 rel2 = colis.touch2(manifold_index) - colis.incoming->position();

    const glm::vec2 relvel = (colis.incoming->velocity_at(rel2) - colis.current->velocity_at(rel1));
    const glm::vec2 reltouch = colis.touch2(manifold_index) - colis.touch1(manifold_index);

    const glm::vec2 normal_dir = glm::normalize(colis.normal);

    const glm::vec2 orth_vel = glm::dot(normal_dir, relvel) * normal_dir;
    const glm::vec2 tangent_vel = relvel - orth_vel;

    const glm::vec2 force = reltouch * rigidity() + orth_vel * restitution() + tangent_vel * friction();
    const float torque1 = kit::cross2D(rel1, force), torque2 = kit::cross2D(force, rel2);
    return {force, torque1, torque2};
}

void spring_driven_solver2D::solve_and_apply_collision_forces(const collision2D &colis) const
{
    KIT_PERF_FUNCTION()

    glm::vec2 force{0.f};
    float torque1 = 0.f;
    float torque2 = 0.f;
    for (std::size_t i = 0; i < colis.size; i++)
    {
        const auto [f, t1, t2] = compute_collision_forces(colis, i);
        force += f;
        torque1 += t1;
        torque2 += t2;
    }
    force /= colis.size;
    torque1 /= colis.size;
    torque2 /= colis.size;

    colis.current->apply_simulation_force(force);
    colis.current->apply_simulation_torque(torque1);

    colis.incoming->apply_simulation_force(-force);
    colis.incoming->apply_simulation_torque(torque2);
}
} // namespace ppx