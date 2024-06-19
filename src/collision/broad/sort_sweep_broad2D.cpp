#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/sort_sweep_broad2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void sort_sweep_broad2D::on_attach()
{
    m_add_edge = kit::callback<collider2D *>([this](collider2D *collider) {
        m_edges.push_back({collider, end_side::LEFT, FLT_MAX});
        m_edges.push_back({collider, end_side::RIGHT, FLT_MAX});
    });
    m_remove_edge = kit::callback<const collider2D &>([this](const collider2D &collider) {
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

void sort_sweep_broad2D::detect_collisions()
{
    KIT_PERF_FUNCTION()

    update_edges();
    m_eligible.clear();

    for (const edge &edg : m_edges)
        if (edg.end == end_side::LEFT)
        {
            for (collider2D *collider : m_eligible)
                process_collision_st(collider, edg.collider);
            m_eligible.insert(edg.collider);
        }
        else
            m_eligible.erase(edg.collider);
}

void sort_sweep_broad2D::update_edges()
{
    for (edge &edg : m_edges)
    {
        const aabb2D &bbox = edg.collider->bounding_box();
        edg.value = edg.end == end_side::LEFT ? bbox.min.x : bbox.max.x;
    }
    std::sort(m_edges.begin(), m_edges.end());
}
} // namespace ppx