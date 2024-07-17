#pragma once

#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/common/alias.hpp"

namespace ppx
{
class quad_tree_broad2D final : public broad_phase2D
{
  public:
    template <class... QTArgs>
    quad_tree_broad2D(world2D &world, QTArgs &&...qt_args)
        : broad_phase2D(world), m_quad_tree(std::forward<QTArgs>(qt_args)...)
    {
        build_tree_from_scratch();
    }

    const char *name() const override;

    void insert(collider2D *collider);
    void erase(collider2D *collider);

    const ppx::quad_tree &quad_tree() const;
    ppx::quad_tree &quad_tree();

    const ppx::quad_tree::properties &props() const;
    void props(const ppx::quad_tree::properties &props);

    std::uint32_t rebuild_count() const;
    void build_tree_from_scratch();

    float rebuild_time_threshold = 1.5f;
    bool force_square_shape = true;

  private:
    void update_pairs(const std::vector<collider2D *> &to_update) override;
    void update_pairs_st(const std::vector<collider2D *> &to_update);
    void update_pairs_mt(const std::vector<collider2D *> &to_update);

    ppx::quad_tree m_quad_tree;

    aabb2D m_qt_bounds;
    float m_rebuild_timer = 0.f;
    std::uint32_t m_rebuild_count = 0;
    bool m_may_rebuild = false;
};
} // namespace ppx
