#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/brute_force_detection2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/profile/perf.hpp"
#include "kit/multithreading/mt_for_each.hpp"

namespace ppx
{
void brute_force_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()
    if (multithreaded)
        detect_collisions_mt();
    else
        detect_collisions_st();
}
void brute_force_detection2D::detect_collisions_st()
{
    for (std::size_t i = 0; i < world->bodies.size(); i++)
        for (std::size_t j = i + 1; j < world->bodies.size(); j++)
        {
            body2D &body1 = world->bodies[i];
            body2D &body2 = world->bodies[j];
            const collision2D colis = generate_collision(body1, body2);
            if (colis.collided)
            {
                try_enter_or_stay_callback(colis);
                m_collisions.push_back(colis);
            }
            else
                try_exit_callback(body1, body2);
        }
    // DEBUG COLLISION COUNT CHECK GOES HERE
}

void brute_force_detection2D::detect_collisions_mt()
{
    const auto exec = [this](const std::size_t thread_idx, body2D &body1) {
        for (std::size_t j = 0; j < world->bodies.size(); j++)
        {
            body2D &body2 = world->bodies[j];
            const collision2D colis = generate_collision(body1, body2);
            if (colis.collided)
            {
                try_enter_or_stay_callback(colis);
                m_mt_collisions[thread_idx].push_back(colis);
            }
            else
                try_exit_callback(body1, body2);
        }
    };
    kit::mt::for_each(world->bodies, exec, PPX_THREAD_COUNT);
    for (const auto &pairs : m_mt_collisions)
        m_collisions.insert(m_collisions.end(), pairs.begin(), pairs.end());
}
} // namespace ppx