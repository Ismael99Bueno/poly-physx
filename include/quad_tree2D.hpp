#ifndef QUAD_TREE2D_HPP
#define QUAD_TREE2D_HPP

#include "entity_ptr.hpp"
#include <memory>
#include <array>

namespace phys
{
    class quad_tree2D
    {
    public:
        quad_tree2D() = delete;
        quad_tree2D(const alg::vec2 &min,
                    const alg::vec2 &max,
                    std::size_t max_entities = 5,
                    std::uint32_t depth = 0);

        void partitions(std::vector<const std::vector<const_entity_ptr> *> &partitions) const;
        void update(const std::vector<entity2D> &entities);
        void rebuild(const std::vector<entity2D> &entities);

        bool full() const;
        bool rock_bottom() const;

        const geo::aabb2D &aabb() const;
        void aabb(const geo::aabb2D &aabb);

        std::size_t max_entities() const;
        void max_entities(std::size_t max_entities);

        bool partitioned() const;
        const std::vector<const_entity_ptr> &entities() const;

        const std::array<std::unique_ptr<quad_tree2D>, 4> &children() const;
        const quad_tree2D &child(std::size_t index) const;
        const quad_tree2D &operator[](std::size_t index) const;

        static std::uint32_t max_depth();
        static void max_depth(std::uint32_t max_depth);

    private:
        std::array<std::unique_ptr<quad_tree2D>, 4> m_children; // TL, TR, BL, BR
        geo::aabb2D m_aabb;
        std::size_t m_max_entities;
        std::uint32_t m_depth;
        static std::uint32_t s_max_depth;
        bool m_partitioned, m_has_children;
        std::vector<const_entity_ptr> m_entities;

        void add_if_inside(const const_entity_ptr &e);
        void clear();
        void create_children();
        void partition();
        void add_to_children(const const_entity_ptr &e);
    };
}

#endif