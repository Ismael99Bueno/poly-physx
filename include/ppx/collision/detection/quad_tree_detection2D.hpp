#ifndef PPX_QUAD_TREE_DETECTION2D_HPP
#define PPX_QUAD_TREE_DETECTION2D_HPP

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/collision/detection/quad_tree.hpp"

namespace ppx
{
class quad_tree_detection2D : public collision_detection2D
{
  public:
    const quad_tree &qtree() const;
    inline static bool force_square_shape = false;

  private:
    quad_tree m_quad_tree{{-10.f, -10.f}, {10.f, 10.f}};

    void detect_collisions() override;
    void detect_collisions_st(const std::vector<const quad_tree::partition *> &partitions);
    void detect_collisions_mt(const std::vector<const quad_tree::partition *> &partitions);

    void update_quad_tree();
};
} // namespace ppx

#endif