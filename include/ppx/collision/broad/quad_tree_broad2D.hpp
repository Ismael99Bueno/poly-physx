#pragma once

#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/common/alias.hpp"

namespace ppx
{
class quad_tree_broad2D final : public broad_phase2D
{
  public:
    using broad_phase2D::broad_phase2D;
    const ppx::quad_tree &quad_tree() const;
    ppx::quad_tree &quad_tree();

    bool force_square_shape = false;
    bool include_non_dynamic = false;

  private:
    using qtpartition = ppx::quad_tree::partition;
    ppx::quad_tree m_quad_tree;

    void detect_collisions() override;
    void detect_collisions_st(const std::vector<const qtpartition *> &partitions);
    void detect_collisions_mt(const std::vector<const qtpartition *> &partitions);

    void update_quad_tree();
};
} // namespace ppx
