#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/brute_force_broad2D.hpp"
#include "ppx/world2D.hpp"

#include "kit/multithreading/mt_for_each.hpp"

#include <numeric>

namespace ppx
{
const char *brute_force_broad2D::name() const
{
    return "Brute Force";
}

void brute_force_broad2D::detect_collisions()
{
    if (params.multithreading && world.thread_pool)
        detect_collisions_mt();
    else
        detect_collisions_st();
}
void brute_force_broad2D::detect_collisions_st()
{
    KIT_PERF_SCOPE("brute_force_broad2D::detect_collisions_st")
    for (std::size_t i = 0; i < world.colliders.size(); i++)
        for (std::size_t j = i + 1; j < world.colliders.size(); j++)
        {
            collider2D *collider1 = world.colliders[i];
            collider2D *collider2 = world.colliders[j];
            process_collision_st(collider1, collider2);
        }
    // DEBUG COLLISION COUNT CHECK GOES HERE
}

void brute_force_broad2D::detect_collisions_mt()
{
    static std::vector<std::size_t> indices;
    indices.resize(world.colliders.size());
    std::iota(indices.begin(), indices.end(), 0);

    kit::mt::for_each(
        *world.thread_pool, indices,
        [this](const std::size_t workload_index, const std::size_t i) {
            for (std::size_t j = i + 1; j < world.colliders.size(); j++)
            {
                collider2D *collider1 = world.colliders[i];
                collider2D *collider2 = world.colliders[j];
                process_collision_mt(collider1, collider2, workload_index);
            }
        },
        params.parallel_workloads);
    join_mt_collisions();
}
} // namespace ppx