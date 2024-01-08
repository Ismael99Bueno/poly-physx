#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/grid_detection2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/container/hashable_tuple.hpp"
#include "kit/multithreading/mt_for_each.hpp"

namespace ppx
{
void grid_detection2D::on_attach()
{
    m_add_cell = kit::callback<body2D &>{[this](body2D &body) {
        const body2D::ptr bptr = body.as_ptr();
        m_cells.push_back({bptr, SIZE_MAX});
    }};
    m_remove_cell = kit::callback<std::size_t>{[this](std::size_t index) {
        for (auto it = m_cells.begin(); it != m_cells.end();)
            if (!it->body)
                it = m_cells.erase(it);
            else
                ++it;
    }};

    world->events.on_body_addition += m_add_cell;
    world->events.on_late_body_removal += m_remove_cell;

    for (body2D &body : world->bodies)
        m_add_cell(body);
}

grid_detection2D::~grid_detection2D()
{
    world->events.on_body_addition -= m_add_cell;
    world->events.on_late_body_removal -= m_remove_cell;
}

void grid_detection2D::detect_collisions_mt()
{
    const auto exec = [this](const std::size_t thread_idx, const std::size_t &start) {
        for (std::size_t i = start; i < m_cells.size(); i++)
            if (m_cells[start].cell_index == m_cells[i].cell_index)
            {
                body2D &body1 = world->bodies[i];
                for (std::size_t j = i + 1; j < m_cells.size(); i++)
                    if (m_cells[start].cell_index == m_cells[j].cell_index)
                    {
                        body2D &body2 = world->bodies[j];
                        process_collision_mt(body1, body2, thread_idx);
                    }
            }
    };
    kit::mt::for_each<PPX_THREAD_COUNT>(m_start_indices, exec);
    join_mt_collisions();
}

void grid_detection2D::detect_collisions_st()
{
    update_cells();
    update_start_indices_from_cells();

    for (const std::size_t &start : m_start_indices)
        for (std::size_t i = start; i < m_cells.size(); i++)
            if (m_cells[start].cell_index == m_cells[i].cell_index)
            {
                body2D &body1 = world->bodies[i];
                for (std::size_t j = i + 1; j < m_cells.size(); i++)
                    if (m_cells[start].cell_index == m_cells[j].cell_index)
                    {
                        body2D &body2 = world->bodies[j];
                        process_collision_st(body1, body2);
                    }
            }
}

std::size_t grid_detection2D::cell_index_from_position(const glm::vec2 &position)
{
    const std::size_t i = (std::size_t)(position.x / cell_size);
    const std::size_t j = (std::size_t)(position.y / cell_size);
    const kit::non_commutative_tuple<std::size_t, std::size_t> hash{i, j};
    return hash() % world->bodies.size();
}

void grid_detection2D::update_cells() // Put this on a callback
{
    for (cell &c : m_cells)
        c.cell_index = cell_index_from_position(c.body->position());

    std::sort(m_cells.begin(), m_cells.end());
}

void grid_detection2D::update_start_indices_from_cells()
{
    m_start_indices.clear();
    std::size_t last_index = m_cells[0].cell_index;
    m_start_indices.push_back(0);
    for (std::size_t i = 1; i < m_cells.size(); i++)
        if (last_index != m_cells[i].cell_index)
        {
            last_index = m_cells[i].cell_index;
            m_start_indices.push_back(i);
        }
}
} // namespace ppx