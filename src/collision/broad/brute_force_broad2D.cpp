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

void brute_force_broad2D::update_pairs(const std::vector<collider2D *> &to_update)
{
    if (params.multithreading && world.thread_pool)
        update_pairs_mt(to_update);
    else
        update_pairs_st(to_update);
}
void brute_force_broad2D::update_pairs_st(const std::vector<collider2D *> &to_update)
{
    for (std::size_t i = 0; i < to_update.size(); i++)
        for (std::size_t j = i + 1; j < world.colliders.size(); j++)
        {
            collider2D *collider1 = to_update[i];
            collider2D *collider2 = world.colliders[j];
            try_create_pair(collider1, collider2);
        }
}

void brute_force_broad2D::update_pairs_mt(const std::vector<collider2D *> &to_update)
{
    auto pool = world.thread_pool;
    const auto lambda = [this](const std::size_t workload_index, collider2D *collider1) {
        for (collider2D *collider2 : world.colliders)
            try_create_pair(collider1, collider2);
    };
    kit::mt::for_each(*pool, to_update, lambda, pool->thread_count());
}
} // namespace ppx