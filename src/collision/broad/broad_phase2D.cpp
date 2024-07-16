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

    // i guess i could multithread this
    KIT_PERF_SCOPE("broad_phase2D::update_pairs");
    {
        KIT_PERF_SCOPE("broad_phase2D::remove_old_pairs");
        for (const collider2D *collider : m_to_update)
            remove_pairs_containing(collider);
    }

    m_pair_flags.resize(m_to_update.size() * world.colliders.size());
    m_pair_flags.assign(m_pair_flags.size(), false);

    update_pairs(m_to_update);
    clear_pending_updates();
    return m_pairs;
}

void broad_phase2D::flag_update(collider2D *collider)
{
    if (!collider->meta.broad_flag)
    {
        collider->meta.broad_flag = true;
        collider->meta.broad_index = m_to_update.size();
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

static std::mutex mutex;

void broad_phase2D::try_create_pair(collider2D *collider1, collider2D *collider2)
{
    const bool flipped = collider1->meta.index > collider2->meta.index;
    const std::size_t idx1 = collider1->meta.broad_index;
    const std::size_t idx2 = collider2->meta.index;
    const std::size_t idx = idx1 * world.colliders.size() + idx2;

    if ((flipped && collider2->meta.broad_flag) || m_pair_flags[idx] || !is_potential_pair(collider1, collider2))
        return;

    if (flipped)
        std::swap(collider1, collider2);

    std::scoped_lock lock(mutex);
    m_pair_flags[idx] = true;
    m_pairs.emplace_back(collider1, collider2);
}

void broad_phase2D::remove_pairs_containing(const collider2D *collider)
{
    for (auto it = m_pairs.begin(); it != m_pairs.end();)
        if (it->first == collider || it->second == collider)
            it = m_pairs.erase(it);
        else
            ++it;
}

} // namespace ppx