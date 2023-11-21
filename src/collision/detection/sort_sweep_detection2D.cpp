#include "ppx/internal/pch.hpp"
#include "ppx/collision/sort_sweep_detection2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void sort_sweep_detection2D::on_attach()
{
    m_add_edge = kit::callback<const body2D::ptr &>([this](const body2D::ptr &body) {
        m_edges.emplace_back(body, edge::end_side::LEFT);
        m_edges.emplace_back(body, edge::end_side::RIGHT);
    });
    m_remove_edge = kit::callback<std::size_t>([this](std::size_t index) {
        for (auto it = m_edges.begin(); it != m_edges.end();)
            if (!it->body)
                it = m_edges.erase(it);
            else
                ++it;
    });

    m_world->events.on_body_addition += m_add_edge;
    m_world->events.on_late_body_removal += m_remove_edge;

    for (std::size_t i = 0; i < m_world->size(); i++)
        m_add_edge((*m_world)[i]);
}

sort_sweep_detection2D::~sort_sweep_detection2D()
{
    m_world->events.on_body_addition -= m_add_edge;
    m_world->events.on_late_body_removal -= m_remove_edge;
}

const std::vector<collision2D> &sort_sweep_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()
    std::unordered_set<const body2D *> eligible;
    sort_edges();

    eligible.reserve(30);
    for (const edge &edg : m_edges)
        if (edg.end == edge::end_side::LEFT)
        {
            for (const body2D *body : eligible)
            {
                collision2D c;
                const body2D &body1 = *body, &body2 = *edg.body;
                if (gather_collision_data(body1, body2, &c))
                {
                    try_enter_or_stay_callback(c);
                    m_collisions.push_back(c);
                }
                else
                    try_exit_callback(body1, body2);
            }
            eligible.insert(edg.body.raw());
        }
        else
            eligible.erase(edg.body.raw());
    return m_collisions;
}

void sort_sweep_detection2D::sort_edges()
{
    const auto cmp = [](const edge &edge1, const edge &edge2) { return edge1.value() < edge2.value(); };
    std::sort(m_edges.begin(), m_edges.end(), cmp);
}
} // namespace ppx