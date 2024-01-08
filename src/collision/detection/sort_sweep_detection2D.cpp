#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/sort_sweep_detection2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void sort_sweep_detection2D::on_attach()
{
    m_add_edge = kit::callback<body2D &>([this](body2D &body) {
        const body2D::ptr bptr = body.as_ptr();
        m_edges.push_back({bptr, end_side::LEFT, 0.f});
        m_edges.push_back({bptr, end_side::RIGHT, 0.f});
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
        m_add_edge(world->bodies[i]);
}

sort_sweep_detection2D::~sort_sweep_detection2D()
{
    world->events.on_body_addition -= m_add_edge;
    world->events.on_late_body_removal -= m_remove_edge;
}

void sort_sweep_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()

    update_edges();
    m_eligible.clear();

    for (const edge &edg : m_edges)
        if (edg.end == end_side::LEFT)
        {
            for (body2D *body : m_eligible)
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
            m_eligible.insert(edg.body.raw());
        }
        else
            m_eligible.erase(edg.body.raw());
}

void sort_sweep_detection2D::update_edges()
{
    for (edge &edg : m_edges)
    {
        const geo::aabb2D &bbox = edg.body->shape().bounding_box();
        edg.value = edg.end == end_side::LEFT ? bbox.min.x : bbox.max.x;
    }
    std::sort(m_edges.begin(), m_edges.end());
}
} // namespace ppx