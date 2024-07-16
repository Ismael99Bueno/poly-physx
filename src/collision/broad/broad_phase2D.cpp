#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
broad_phase2D::broad_phase2D(world2D &world) : worldref2D(world)
{
    m_to_update.insert(m_to_update.end(), world.colliders.begin(), world.colliders.end());
}

const char *broad_phase2D::name() const
{
    return "Unnamed";
}

const std::vector<broad_phase2D::cpair> &broad_phase2D::update_pairs()
{
    if (m_to_update.empty() || world.rk_subset_index() != 0)
        return m_pairs;

    auto pool = world.thread_pool;
    if (params.multithreading && pool)
    {
        m_mt_pairs.resize(pool->thread_count());
        for (auto &pairs : m_mt_pairs)
            pairs.clear();
    }

    // i guess i could multithread this
    for (const collider2D *collider : m_to_update)
        remove_pairs_containing(collider);

    update_pairs(m_to_update);
    clear_pending_updates();
    if (params.multithreading)
        for (auto &pairs : m_mt_pairs)
            m_pairs.insert(m_pairs.end(), pairs.begin(), pairs.end());
    return m_pairs;
}

void broad_phase2D::flag_update(collider2D *collider)
{
    if (!collider->meta.broad_flag)
    {
        collider->meta.broad_flag = true;
        m_to_update.push_back(collider);
    }
}
void broad_phase2D::clear_pending_updates()
{
    for (collider2D *collider : m_to_update)
        collider->meta.broad_flag = false;
    m_to_update.clear();
}

std::size_t broad_phase2D::pending_updates() const
{
    return m_to_update.size();
}

const std::vector<broad_phase2D::cpair> &broad_phase2D::pairs() const
{
    return m_pairs;
}

static bool is_potential_pair(const collider2D *collider1, const collider2D *collider2) // this must be tuned
{
    const body2D *body1 = collider1->body();
    const body2D *body2 = collider2->body();
    return body1 != body2 && geo::intersects(collider1->fat_bbox(), collider2->fat_bbox());
}

void broad_phase2D::try_create_pair_st(collider2D *collider1, collider2D *collider2)
{
    const bool flipped = collider1->meta.index > collider2->meta.index;
    if (flipped && collider2->meta.broad_flag)
        return;
    if (is_potential_pair(collider1, collider2))
    {
        if (flipped)
            std::swap(collider1, collider2);
        m_pairs.emplace_back(collider1, collider2);
    }
}

void broad_phase2D::remove_pairs_containing(const collider2D *collider)
{
    for (auto it = m_pairs.begin(); it != m_pairs.end();)
        if (it->first == collider || it->second == collider)
            it = m_pairs.erase(it);
        else
            ++it;
}

void broad_phase2D::try_create_pair_mt(collider2D *collider1, collider2D *collider2, const std::size_t thread_index)
{
    const bool flipped = collider1->meta.index > collider2->meta.index;
    if (flipped && collider2->meta.broad_flag)
        return;
    if (is_potential_pair(collider1, collider2))
    {
        if (flipped)
            std::swap(collider1, collider2);
        m_mt_pairs[thread_index].emplace_back(collider1, collider2);
    }
}
} // namespace ppx