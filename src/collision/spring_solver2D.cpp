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
    const glm::vec2 rel1 = colis.touch1 - colis.current->position(), rel2 = colis.touch2 - colis.incoming->position();
    const glm::vec2 vel1 = colis.current->velocity_at(rel1), vel2 = colis.incoming->velocity_at(rel2);
    const glm::vec2 force = stiffness * (colis.touch2 - colis.touch1) + dampening * (vel2 - vel1);

    const float torque1 = kit::cross2D(rel1, force), torque2 = kit::cross2D(force, rel2);
    if (colis.current->kinematic)
    {
        colis.current->apply_simulation_force(force);
        colis.current->apply_simulation_torque(torque1);
    }
    if (colis.incoming->kinematic)
    {
        colis.incoming->apply_simulation_force(-force);
        colis.incoming->apply_simulation_torque(torque2);
    }
}

std::array<float, 6> spring_solver2D::forces_upon_collision(const collision2D &colis) const
{
    KIT_PERF_FUNCTION()
    const glm::vec2 rel1 = colis.touch1 - colis.current->position(), rel2 = colis.touch2 - colis.incoming->position();
    const glm::vec2 vel1 = colis.current->velocity_at(rel1), vel2 = colis.incoming->velocity_at(rel2);
    const glm::vec2 force = stiffness * (colis.touch2 - colis.touch1) + dampening * (vel2 - vel1);

    const float torque1 = kit::cross2D(rel1, force), torque2 = kit::cross2D(force, rel2);
    return {force.x, force.y, torque1, -force.x, -force.y, torque2};
}
} // namespace ppx