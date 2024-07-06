#pragma once

#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/common/alias.hpp"

namespace ppx
{
class quad_tree_broad2D final : public broad_phase2D
{
  public:
    quad_tree_broad2D(world2D &world);
    ~quad_tree_broad2D();

    const char *name() const override;

    void insert(collider2D *collider);
    void erase(collider2D *collider);

    const ppx::quad_tree &quad_tree() const;
    ppx::quad_tree &quad_tree();

    glm::vec2 enlargement_vector_from_velocity(const glm::vec2 &velocity) const;
    std::uint32_t rebuild_count() const;

    float bounding_box_anticipation = 0.1f;
    float rebuild_time_threshold = 5.f;
    bool force_square_shape = true;

  private:
    using qtpartition = ppx::quad_tree::partition;

    void detect_collisions() override;
    void detect_collisions_st();
    void detect_collisions_mt();

    void build_tree_from_scratch();

    ppx::quad_tree m_quad_tree;
    std::vector<qtpartition> m_partitions;

    aabb2D m_qt_bounds;
    float m_rebuild_timer = 0.f;
    std::uint32_t m_rebuild_count = 0;
    bool m_needs_partition_update = true;
    bool m_may_rebuild = false;
};
} // namespace ppx
