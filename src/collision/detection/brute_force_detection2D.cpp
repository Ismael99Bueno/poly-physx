#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/brute_force_detection2D.hpp"
#include "ppx/world2D.hpp"

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
    for (std::size_t i = 0; i < world.colliders.size(); i++)
        for (std::size_t j = i + 1; j < world.colliders.size(); j++)
        {
            collider2D *collider1 = world.colliders[i];
            collider2D *collider2 = world.colliders[j];
            process_collision_st(collider1, collider2);
        }
    // DEBUG COLLISION COUNT CHECK GOES HERE
}

void brute_force_detection2D::detect_collisions_mt()
{
    kit::mt::for_each(PPX_THREAD_COUNT, world.colliders, [this](const std::size_t thread_idx, collider2D *collider1) {
        for (std::size_t j = 0; j < world.colliders.size(); j++)
        {
            collider2D *collider2 = world.colliders[j];
            process_collision_mt(collider1, collider2, thread_idx);
        }
    });
    join_mt_collisions();
}
} // namespace ppx