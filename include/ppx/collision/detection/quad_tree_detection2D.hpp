#pragma once

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/collision/detection/quad_tree.hpp"

namespace ppx
{
class quad_tree_detection2D final : public collision_detection2D
{
  public:
    using collision_detection2D::collision_detection2D;
    const quad_tree &qtree() const;
    bool force_square_shape = false;

  private:
    quad_tree m_quad_tree{{-10.f, -10.f}, {10.f, 10.f}};
    kit::mt::feach_thread_pool<const std::vector<const quad_tree::partition *>> m_pool{PPX_THREAD_COUNT};

    void detect_collisions() override;
    void detect_collisions_st(const std::vector<const quad_tree::partition *> &partitions);
    void detect_collisions_mt(const std::vector<const quad_tree::partition *> &partitions);

    void update_quad_tree();
};
} // namespace ppx
