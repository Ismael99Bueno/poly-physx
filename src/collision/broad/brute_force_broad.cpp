#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/brute_force_broad.hpp"
#include "ppx/world.hpp"
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
    m_new_pairs_count = 0;
    if (params.multithreading && world.thread_pool)
        update_pairs_mt(to_update);
    else
        update_pairs_st(to_update);
}
void brute_force_broad2D::update_pairs_st(const std::vector<collider2D *> &to_update)
{
    KIT_PERF_SCOPE("ppx::brute_force_broad2D::update_pairs_st")
    for (std::size_t i = 0; i < to_update.size(); i++)
        for (std::size_t j = i + 1; j < world.colliders.size(); j++)
        {
            collider2D *collider1 = to_update[i];
            collider2D *collider2 = world.colliders[j];
            if (is_potential_new_pair(&collider1, &collider2))
            {
                m_pairs.emplace_back(collider1, collider2);
                m_unique_pairs.emplace(collider1, collider2);
                m_new_pairs_count++;
            }
        }
}

void brute_force_broad2D::update_pairs_mt(const std::vector<collider2D *> &to_update)
{
    KIT_PERF_SCOPE("ppx::brute_force_broad2D::update_pairs_mt")
    const auto pool = world.thread_pool;
    auto lambda = [this](auto it1, auto it2) {
        thread_local std::vector<pair> new_pairs;
        new_pairs.clear();

        for (auto it = it1; it != it2; ++it)
            for (collider2D *collider2 : world.colliders)
            {
                collider2D *collider1 = *it;
                if (is_potential_new_pair(&collider1, &collider2))
                    new_pairs.emplace_back(collider1, collider2);
            }

        return new_pairs;
    };
    auto futures = kit::mt::for_each_iter(*pool, to_update.begin(), to_update.end(), lambda, pool->thread_count());
    const std::size_t start_idx = m_pairs.size();

    for (auto &f : futures)
    {
        const auto new_pairs = f.get();
        m_pairs.insert(m_pairs.end(), new_pairs.begin(), new_pairs.end());
        m_new_pairs_count += new_pairs.size();
    }
    for (auto it = m_pairs.begin() + start_idx; it != m_pairs.end(); ++it)
        m_unique_pairs.emplace(it->collider1, it->collider2);
}
} // namespace ppx