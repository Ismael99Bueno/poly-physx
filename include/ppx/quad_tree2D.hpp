#ifndef QUAD_TREE2D_HPP
#define QUAD_TREE2D_HPP
#include "ppx/core.hpp"

#include "ppx/entity2D_ptr.hpp"
#include <memory>
#include <array>

namespace ppx
{
    class quad_tree2D final
    {
    public:
        quad_tree2D(const glm::vec2 &min,
                    const glm::vec2 &max,
                    std::size_t max_entities = 12,
                    std::uint32_t depth = 0);

        void partitions(std::vector<const std::vector<const entity2D *> *> &partitions) const;
        void insert(const entity2D *e);
        void clear();

        const geo::aabb2D &aabb() const;
        void aabb(const geo::aabb2D &aabb);

        std::size_t max_entities() const;
        void max_entities(std::size_t max_entities);

        bool partitioned() const;
        const std::vector<const entity2D *> &entities() const;

        const std::array<scope<quad_tree2D>, 4> &children() const;
        const quad_tree2D &child(std::size_t index) const;
        const quad_tree2D &operator[](std::size_t index) const;

        static std::uint32_t max_depth();
        static void max_depth(std::uint32_t max_depth);

    private:
        std::array<scope<quad_tree2D>, 4> m_children = {nullptr, nullptr, nullptr, nullptr}; // TL, TR, BL, BR
        geo::aabb2D m_aabb;
        std::size_t m_max_entities;
        std::uint32_t m_depth;
        static std::uint32_t s_max_depth;
        bool m_partitioned = false, m_has_children = false;
        std::vector<const entity2D *> m_entities;

        bool full() const;
        bool rock_bottom() const;
        void create_children();
        void reset_children();
        void partition();
        void insert_to_children(const entity2D *e);

        quad_tree2D(quad_tree2D &&) = default;
        quad_tree2D &operator=(quad_tree2D &&) = default;
    };
}

#endif