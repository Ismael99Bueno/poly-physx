#pragma once

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class sort_sweep_detection2D final : public collision_detection2D
{
    enum class end_side
    {
        LEFT,
        RIGHT
    };
    struct edge
    {
        collider2D *collider;
        end_side end;
        float value;
        bool operator<(const edge &other) const
        {
            return value < other.value;
        }
    };

  public:
    using collision_detection2D::collision_detection2D;
    ~sort_sweep_detection2D();

  private:
    std::vector<edge> m_edges;
    std::unordered_set<collider2D *> m_eligible;

    kit::callback<collider2D *> m_add_edge;
    kit::callback<const collider2D &> m_remove_edge;

    void detect_collisions() override;
    void on_attach() override;
    void update_edges();
};
} // namespace ppx
