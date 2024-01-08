#pragma once

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class sort_sweep_detection2D final : public collision_detection2D, kit::non_copyable
{
    enum class end_side
    {
        LEFT,
        RIGHT
    };
    struct edge
    {
        body2D::ptr body;
        end_side end;
        float value;
        bool operator<(const edge &other) const
        {
            return value < other.value;
        }
    };

  public:
    ~sort_sweep_detection2D();

  private:
    std::vector<edge> m_edges;
    std::unordered_set<body2D *> m_eligible;

    kit::callback<body2D &> m_add_edge;
    kit::callback<std::size_t> m_remove_edge;

    void detect_collisions() override;
    void on_attach() override;
    void update_edges();
};
} // namespace ppx
