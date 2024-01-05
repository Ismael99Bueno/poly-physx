#pragma once

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class sort_sweep_detection2D final : public collision_detection2D, kit::non_copyable
{
  public:
    struct edge
    {
      public:
        enum class end_side
        {
            LEFT,
            RIGHT
        };

        edge(const body2D::ptr &body, end_side end) : body(body), end(end)
        {
        }
        body2D::ptr body;
        end_side end;

        float value() const
        {
            const geo::aabb2D &bbox = body->shape().bounding_box();
            return end == end_side::LEFT ? bbox.min.x : bbox.max.x;
        }
    };

    ~sort_sweep_detection2D();

  private:
    std::vector<edge> m_edges;
    kit::callback<body2D &> m_add_edge;
    kit::callback<std::size_t> m_remove_edge;

    void detect_collisions() override;
    void on_attach() override;
    void sort_edges();
};
} // namespace ppx
