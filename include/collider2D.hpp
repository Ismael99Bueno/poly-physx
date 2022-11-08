#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "entity2D.hpp"
#include "entity_ptr.hpp"
#include "constrain2D.hpp"
#include <vector>
#include <utility>

namespace physics
{
    class collider2D
    {
    public:
        collider2D() = delete;
        collider2D(const std::vector<entity2D> &entities, std::size_t allocations = 40);

        void add(std::size_t index); // TODO: Implement remove
        void detect_collisions();

    private:
        struct collision_pair
        {
        public:
            collision_pair() = delete;
            collision_pair(const const_entity_ptr &e1, const const_entity_ptr &e2);
            const_entity_ptr e1, e2;
        };

        struct interval
        {
        public:
            enum end
            {
                LOWER,
                HIGHER
            };

            interval(const const_entity_ptr &e, end end_type);

            const const_entity_ptr &entity() const;
            float value() const;
            end type() const;

        private:
            const_entity_ptr m_entity;
            end m_end;
        };

        class collision_constrain : public constrain2D<2>
        {
        public:
            using constrain2D<2>::constrain2D;

        private:
            const vec2 *m_vx1, *m_vx2;
            float constrain(const std::array<const_entity_ptr, 2> &entities) const override;
            float constrain_derivative(const std::array<const_entity_ptr, 2> &entities) const override;

            void compute_closest_vertices();
            std::pair<vec2, vec2> compute_vertices_velocities() const;
        };

        const std::vector<entity2D> &m_buffer;
        std::vector<const_entity_ptr> m_entities;
        std::vector<interval> m_intervals;
        std::vector<collision_pair> m_collisions;

        void sort_entities();
    };
}

#endif