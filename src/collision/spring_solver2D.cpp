#include "ppx/internal/pch.hpp"
#include "ppx/collision/spring_solver2D.hpp"
#include "kit/profile/perf.hpp"

#include "kit/utility/multithreading.hpp"

#if defined(PPX_MULTITHREADED) && defined(KIT_PROFILE)
#pragma message(                                                                                                       \
        "Multithreading for PPX will be disabled because the thread unsafe profiling features of cpp-kit are enabled")
#undef PPX_MULTITHREADED
#endif

namespace ppx
{
static float cross(const glm::vec2 &v1, const glm::vec2 &v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

spring_solver2D::spring_solver2D(const float stiffness, const float dampening)
    : stiffness(stiffness), dampening(dampening)
{
}

void spring_solver2D::solve(const std::vector<collision2D> &collisions, std::vector<float> &state_derivative) const
{
#ifdef PPX_MULTITHREADED
    kit::const_for_each_mt<PPX_THREAD_COUNT, collision2D>(
        collisions, [this, &state_derivative](const std::size_t thread_index, const collision2D &colis) {
            if (colis.valid)
                solve(colis, state_derivative);
        });
#else
    for (const collision2D &colis : collisions)
        if (colis.valid)
            solve(colis, state_derivative);
#endif
}

void spring_solver2D::solve(const collision2D &colis, std::vector<float> &state_derivative) const
{
    KIT_PERF_FUNCTION()
    const std::array<float, 6> forces = forces_upon_collision(colis);
    for (std::size_t i = 0; i < 3; i++)
    {
        if (colis.current->kinematic)
            state_derivative[colis.current->index * 6 + i + 3] += forces[i];
        if (colis.incoming->kinematic)
            state_derivative[colis.incoming->index * 6 + i + 3] += forces[i + 3];
    }
}

std::array<float, 6> spring_solver2D::forces_upon_collision(const collision2D &colis) const
{
    KIT_PERF_FUNCTION()
    const glm::vec2 rel1 = colis.touch1 - colis.current->transform().position,
                    rel2 = colis.touch2 - colis.incoming->transform().position;
    const glm::vec2 vel1 = colis.current->velocity_at(rel1), vel2 = colis.incoming->velocity_at(rel2);
    const glm::vec2 force = stiffness * (colis.touch2 - colis.touch1) + dampening * (vel2 - vel1);

    const float torque1 = cross(rel1, force), torque2 = cross(force, rel2);
    return {force.x, force.y, torque1, -force.x, -force.y, torque2};
}
} // namespace ppx