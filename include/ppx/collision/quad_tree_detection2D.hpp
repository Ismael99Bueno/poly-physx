#ifndef PPX_QUAD_TREE_DETECTION2D_HPP
#define PPX_QUAD_TREE_DETECTION2D_HPP

#include "ppx/collision/collision_detection2D.hpp"
#include "ppx/collision/quad_tree2D.hpp"

namespace ppx
{
class quad_tree_detection2D : public collision_detection2D
{
  public:
    const std::vector<collision2D> &detect_collisions() override;

  private:
    quad_tree2D m_quad_tree;

    const std::vector<collision2D> &detect_collisions_st(const std::vector<const quad_tree2D::partition *> &partitions);
    const std::vector<collision2D> &detect_collisions_mt(const std::vector<const quad_tree2D::partition *> &partitions);

    void update_quad_tree();
};
} // namespace ppx

#endif