#pragma once

#include "ppx/entities/collider2D.hpp"
#include "kit/memory/scope.hpp"
#include <memory>
#include <array>

namespace ppx
{
class quad_tree
{
  public:
    quad_tree(const glm::vec2 &min, const glm::vec2 &max, std::size_t max_colliders = 12, std::uint32_t depth = 0);
    using partition = std::vector<collider2D *>;

    aabb2D aabb;

    void collect_partitions(std::vector<const partition *> &partitions) const;
    void insert(collider2D *collider);
    void clear();

    bool partitioned() const;
    const partition &colliders() const;

    const std::array<kit::scope<quad_tree>, 4> &children() const;
    const quad_tree &child(std::size_t index) const;
    const quad_tree &operator[](std::size_t index) const;

    inline static std::size_t max_colliders = 12;
    inline static std::uint32_t max_depth = 12;
    inline static float min_size = 14.f;

  private:
    std::array<kit::scope<quad_tree>, 4> m_children = {nullptr, nullptr, nullptr, nullptr}; // TL, TR, BL, BR

    std::uint32_t m_depth;
    bool m_partitioned = false, m_has_children = false;
    partition m_colliders;

    bool full() const;
    bool rock_bottom() const;
    void create_children();
    void reset_children();
    void subdivide();
    void insert_to_children(collider2D *collider);

    quad_tree(quad_tree &&) = default;
    quad_tree &operator=(quad_tree &&) = default;
};
} // namespace ppx
