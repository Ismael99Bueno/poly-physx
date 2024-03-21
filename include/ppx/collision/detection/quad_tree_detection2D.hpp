#pragma once

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "kit/container/quad_tree.hpp"

namespace ppx
{
class quad_tree_detection2D final : public collision_detection2D
{
  public:
    using collision_detection2D::collision_detection2D;
    const kit::quad_tree<collider2D *> &quad_tree() const;
    kit::quad_tree<collider2D *> &quad_tree();
    bool force_square_shape = false;

  private:
    using qtpartition = kit::quad_tree<collider2D *>::partition;
    kit::quad_tree<collider2D *> m_quad_tree;

    void detect_collisions() override;
    void detect_collisions_st(const std::vector<const qtpartition *> &partitions);
    void detect_collisions_mt(const std::vector<const qtpartition *> &partitions);

    void update_quad_tree();
};
} // namespace ppx
