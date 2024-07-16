#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/sort_sweep_broad2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void sort_sweep_broad2D::on_attach()
{
    m_add_edge = kit::callback<collider2D *>([this](collider2D *collider) {
        m_edges.push_back({collider, end_side::LOWER, FLT_MAX});
        m_edges.push_back({collider, end_side::UPPER, FLT_MAX});
    });
    m_remove_edge = kit::callback<collider2D &>([this](collider2D &collider) {
        for (auto it = m_edges.begin(); it != m_edges.end();)
            if (it->collider == &collider)
                it = m_edges.erase(it);
            else
                ++it;
    });

    world.colliders.events.on_addition += m_add_edge;
    world.colliders.events.on_removal += m_remove_edge;

    for (collider2D *collider : world.colliders)
        m_add_edge(collider);
}

sort_sweep_broad2D::~sort_sweep_broad2D()
{
    world.colliders.events.on_addition -= m_add_edge;
    world.colliders.events.on_removal -= m_remove_edge;
}

const char *sort_sweep_broad2D::name() const
{
    return "Sort and Sweep";
}

void sort_sweep_broad2D::update_pairs(const std::vector<collider2D *> &to_update)
{
    update_edges();
    if (params.multithreading)
        update_pairs_mt(to_update);
    else
        update_pairs_st(to_update);
}

void sort_sweep_broad2D::update_pairs_st(const std::vector<collider2D *> &to_update)
{
    static std::unordered_set<collider2D *> eligible;
    static std::unordered_set<collider2D *> to_update_set;

    eligible.clear();
    to_update_set.clear();
    to_update_set.insert(to_update.begin(), to_update.end());
    std::uint32_t relevant_count = 0;

    for (const edge &edg : m_edges)
    {
        const bool contained = to_update_set.contains(edg.collider);
        if (edg.end == end_side::LOWER)
        {
            relevant_count += contained;
            if (relevant_count > 0)
                for (collider2D *collider : eligible)
                    try_create_pair(edg.collider, collider);
            eligible.insert(edg.collider);
        }
        else
        {
            eligible.erase(edg.collider);
            relevant_count -= contained;
        }
    }
}

void sort_sweep_broad2D::update_pairs_mt(const std::vector<collider2D *> &to_update)
{
    update_pairs_st(to_update);
}

void sort_sweep_broad2D::update_edges()
{
    KIT_PERF_SCOPE("sort_sweep_broad2D::update_edges")
    for (edge &edg : m_edges)
    {
        const aabb2D &bbox = edg.collider->fat_bbox();
        edg.value = edg.end == end_side::LOWER ? bbox.min.x : bbox.max.x;
    }
    std::sort(m_edges.begin(), m_edges.end());
}
} // namespace ppx