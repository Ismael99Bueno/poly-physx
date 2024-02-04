#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/sort_sweep_detection2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void sort_sweep_detection2D::on_attach()
{
    m_add_edge = kit::callback<collider2D &>([this](collider2D &collider) {
        const collider2D::ptr bptr = collider.as_ptr();
        m_edges.push_back({bptr, end_side::LEFT, FLT_MAX});
        m_edges.push_back({bptr, end_side::RIGHT, FLT_MAX});
    });
    m_remove_edge = kit::callback<std::size_t>([this](std::size_t index) {
        for (auto it = m_edges.begin(); it != m_edges.end();)
            if (!it->collider)
                it = m_edges.erase(it);
            else
                ++it;
    });

    world.colliders.events.on_addition += m_add_edge;
    world.colliders.events.on_late_removal += m_remove_edge;

    for (collider2D &collider : world.colliders)
        m_add_edge(collider);
}

sort_sweep_detection2D::~sort_sweep_detection2D()
{
    world.colliders.events.on_addition -= m_add_edge;
    world.colliders.events.on_late_removal -= m_remove_edge;
}

void sort_sweep_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()

    update_edges();
    m_eligible.clear();

    for (const edge &edg : m_edges)
        if (edg.end == end_side::LEFT)
        {
            for (collider2D *collider : m_eligible)
            {
                collider2D &collider1 = *collider;
                collider2D &collider2 = *edg.collider;
                process_collision_st(collider1, collider2);
            }
            m_eligible.insert(edg.collider.raw());
        }
        else
            m_eligible.erase(edg.collider.raw());
}

void sort_sweep_detection2D::update_edges()
{
    for (edge &edg : m_edges)
    {
        const aabb2D &bbox = edg.collider->shape().bounding_box();
        edg.value = edg.end == end_side::LEFT ? bbox.min.x : bbox.max.x;
    }
    std::sort(m_edges.begin(), m_edges.end());
}
} // namespace ppx