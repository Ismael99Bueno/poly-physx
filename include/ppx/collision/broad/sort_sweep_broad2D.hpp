#pragma once

#include "ppx/collision/broad/broad_phase2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class sort_sweep_broad2D final : public broad_phase2D
{
    enum class end_side
    {
        LOWER,
        UPPER
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
    using broad_phase2D::broad_phase2D;
    ~sort_sweep_broad2D();

  private:
    std::vector<edge> m_edges;

    kit::callback<collider2D *> m_add_edge;
    kit::callback<collider2D &> m_remove_edge;

    void detect_collisions() override;
    void on_attach() override;
    void update_edges();
};
} // namespace ppx
