#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
broad_phase2D::broad_phase2D(world2D &world) : worldref2D(world)
{
    for (collider2D *collider : world.colliders)
    {
        collider->meta.broad_flag = true;
        m_to_update.push_back(collider);
    }
}

const char *broad_phase2D::name() const
{
    return "Unnamed";
}

const std::vector<broad_phase2D::pair> &broad_phase2D::update_pairs()
{
    if (m_to_update.empty() || world.rk_subset_index() != 0)
        return m_pairs;
    KIT_PERF_SCOPE("broad_phase2D::update_pairs")

    remove_outdated_pairs();
    update_pairs(m_to_update);
    clear_pending_updates();
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

void broad_phase2D::remove_pairs_containing(const collider2D *collider)
{
    for (std::size_t i = m_pairs.size() - 1; i != SIZE_MAX; --i)
    {
        const pair &p = m_pairs[i];
        if (p.collider1 == collider || p.collider2 == collider)
        {
            m_unique_pairs.erase({p.collider1, p.collider2});
            m_pairs.erase(m_pairs.begin() + i);
        }
    }
}

void broad_phase2D::remove_outdated_pairs()
{
    KIT_PERF_SCOPE("broad_phase2D::remove_outdated_pairs")
    std::swap(m_last_pairs, m_pairs);
    m_pairs.clear();
    for (const pair &p : m_last_pairs)
        if (geo::intersects(p.collider1->fat_bbox(), p.collider2->fat_bbox()))
            m_pairs.push_back(p);
        else
            m_unique_pairs.erase({p.collider1, p.collider2});
}

const std::vector<broad_phase2D::pair> &broad_phase2D::pairs() const
{
    return m_pairs;
}

std::size_t broad_phase2D::new_pairs_count() const
{
    return m_new_pairs_count;
}
std::size_t broad_phase2D::pending_updates() const
{
    return m_to_update.size();
}

bool broad_phase2D::is_potential_new_pair(collider2D **collider1, collider2D **collider2) const
{
    collider2D *c1 = *collider1;
    collider2D *c2 = *collider2;
    const body2D *body1 = c1->body();
    const body2D *body2 = c2->body();
    if (body1 == body2)
        return false;
    const bool flipped = c1->meta.index > c2->meta.index;
    if ((flipped && c2->meta.broad_flag) || !geo::intersects(c1->fat_bbox(), c2->fat_bbox()))
        return false;

    if (flipped)
        std::swap(*collider1, *collider2);
    return !m_unique_pairs.contains({*collider1, *collider2});
}

} // namespace ppx