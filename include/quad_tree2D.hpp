#ifndef QUAD_TREE2D_HPP
#define QUAD_TREE2D_HPP

#include "vec2.hpp"
#include "entity_ptr.hpp"
#include <memory>
#include <array>

namespace phys
{
    class quad_tree2D
    {
    public:
        quad_tree2D() = delete;
        quad_tree2D(const alg::vec2 &pos, const alg::vec2 &dim, std::size_t max_entities = 5);

        void partitions(std::vector<const std::vector<const_entity_ptr> *> &partitions) const;
        void update(const std::vector<entity2D> &entities);
        void rebuild(const std::vector<entity2D> &entities);

        bool full() const;

        const alg::vec2 &pos() const;
        const alg::vec2 &dim() const;

        void pos(const alg::vec2 &pos); // quad tree gets invalid when calling these. must call build
        void dim(const alg::vec2 &dim);

        std::size_t max_entities() const;
        void max_entities(std::size_t max_entities);

        bool partitioned() const;
        const std::vector<const_entity_ptr> &entities() const;

        const std::array<std::unique_ptr<quad_tree2D>, 4> &children() const;
        const quad_tree2D &child(std::size_t index) const;
        const quad_tree2D &operator[](std::size_t index) const;

    private:
        std::array<std::unique_ptr<quad_tree2D>, 4> m_children; // TL, TR, BL, BR
        alg::vec2 m_pos, m_dim;
        std::size_t m_max_entities;
        bool m_partitioned, m_has_children;
        std::vector<const_entity_ptr> m_entities;

        void add_if_inside(const const_entity_ptr &e);
        void clear();
        void create_children();
        void partition();
        bool contains(const geo::box2D &bbox) const;
        void add_to_children(const const_entity_ptr &e);
    };
}

#endif