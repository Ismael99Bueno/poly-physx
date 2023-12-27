#include "ppx/internal/pch.hpp"
#include "ppx/collision/resolution/spring_driven_resolution2D.hpp"
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
spring_driven_resolution2D::spring_driven_resolution2D(const float rigidity, const float normal_damping,
                                                       const float tangent_damping)
    : rigidity(rigidity), normal_damping(normal_damping), tangent_damping(tangent_damping)
{
}
void spring_driven_resolution2D::solve(const std::vector<collision2D> &collisions) const
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

std::tuple<glm::vec2, float, float> spring_driven_resolution2D::compute_collision_forces(
    const collision2D &colis, std::size_t manifold_index) const
{
    const glm::vec2 rel1 = colis.touch1(manifold_index) - colis.body1->position();
    const glm::vec2 rel2 = colis.touch2(manifold_index) - colis.body2->position();

    const glm::vec2 relvel = (colis.body2->velocity_at(rel2) - colis.body1->velocity_at(rel1));
    const glm::vec2 reltouch = colis.touch2(manifold_index) - colis.touch1(manifold_index);

    const glm::vec2 normal_dir = glm::normalize(colis.mtv);

    const glm::vec2 normal_vel = glm::dot(normal_dir, relvel) * normal_dir;
    const glm::vec2 tangent_vel = relvel - normal_vel;

    const glm::vec2 force = reltouch * rigidity + normal_vel * normal_damping + tangent_vel * tangent_damping;
    const float torque1 = kit::cross2D(rel1, force), torque2 = kit::cross2D(force, rel2);
    return {force, torque1, torque2};
}

void spring_driven_resolution2D::solve_and_apply_collision_forces(const collision2D &colis) const
{
    KIT_PERF_FUNCTION()

    glm::vec2 force{0.f};
    float torque1 = 0.f;
    float torque2 = 0.f;
    for (std::size_t i = 0; i < colis.manifold.size; i++)
    {
        const auto [f, t1, t2] = compute_collision_forces(colis, i);
        force += f;
        torque1 += t1;
        torque2 += t2;
    }
    force /= colis.manifold.size;
    torque1 /= colis.manifold.size;
    torque2 /= colis.manifold.size;

    colis.body1->apply_simulation_force(force);
    colis.body1->apply_simulation_torque(torque1);

    colis.body2->apply_simulation_force(-force);
    colis.body2->apply_simulation_torque(torque2);
}
} // namespace ppx