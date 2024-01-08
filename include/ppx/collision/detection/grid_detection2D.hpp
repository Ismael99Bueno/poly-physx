#pragma once

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class grid_detection2D final : public collision_detection2D, kit::non_copyable
{
    struct cell
    {
        body2D::ptr body;
        std::size_t cell_index;
        bool operator<(const cell &other) const
        {
            return cell_index < other.cell_index;
        };
    };

  public:
    grid_detection2D(float cell_size = 15.f);
    ~grid_detection2D();

    float cell_size;

  private:
    std::vector<cell> m_cells;
    std::vector<std::size_t> m_start_indices;

    kit::callback<body2D &> m_add_cell;
    kit::callback<std::size_t> m_remove_cell;

    void on_attach() override;
    void detect_collisions() override;

    void detect_collisions_st();
    void detect_collisions_mt();

    std::size_t cell_index_from_position(const glm::vec2 &position);
    void update_cells();
    void update_start_indices_from_cells();
};
} // namespace ppx