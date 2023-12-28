#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/sort_sweep_detection2D.hpp"
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

    world->events.on_body_addition += m_add_edge;
    world->events.on_late_body_removal += m_remove_edge;

    for (std::size_t i = 0; i < world->bodies.size(); i++)
        m_add_edge(world->bodies.ptr(i));
}

sort_sweep_detection2D::~sort_sweep_detection2D()
{
    world->events.on_body_addition -= m_add_edge;
    world->events.on_late_body_removal -= m_remove_edge;
}

void sort_sweep_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()
    std::unordered_set<body2D *> eligible;
    sort_edges();

    eligible.reserve(30);
    for (const edge &edg : m_edges)
        if (edg.end == edge::end_side::LEFT)
        {
            for (body2D *body : eligible)
            {
                body2D &body1 = *body;
                body2D &body2 = *edg.body;
                const collision2D colis = generate_collision(body1, body2);

                if (colis.collided)
                {
                    try_enter_or_stay_callback(colis);
                    m_collisions.push_back(colis);
                }
                else
                    try_exit_callback(body1, body2);
            }
            eligible.insert(edg.body.raw());
        }
        else
            eligible.erase(edg.body.raw());
}

void sort_sweep_detection2D::sort_edges()
{
    const auto cmp = [](const edge &edge1, const edge &edge2) { return edge1.value() < edge2.value(); };
    std::sort(m_edges.begin(), m_edges.end(), cmp);
}
} // namespace ppx