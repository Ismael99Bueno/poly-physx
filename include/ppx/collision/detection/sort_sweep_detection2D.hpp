#ifndef PPX_SORT_SWEEP_DETECTION2D_HPP
#define PPX_SORT_SWEEP_DETECTION2D_HPP

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class sort_sweep_detection2D : public collision_detection2D, kit::non_copyable
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

        edge(const body2D::const_ptr &body, end_side end) : body(body), end(end)
        {
        }
        body2D::const_ptr body;
        end_side end;

        float value() const
        {
            const geo::aabb2D &bbox = body->shape().bounding_box();
            return end == end_side::LEFT ? bbox.min().x : bbox.max().x;
        }
    };

    ~sort_sweep_detection2D();

    const std::vector<collision2D> &detect_collisions() override;

  private:
    std::vector<edge> m_edges;
    kit::callback<const body2D::ptr &> m_add_edge;
    kit::callback<std::size_t> m_remove_edge;

    void on_attach() override;
    void sort_edges();
};
} // namespace ppx

#endif