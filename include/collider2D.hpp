#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "entity2D.hpp"
#include "entity_ptr.hpp"
#include <vector>

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

        const std::vector<entity2D> &m_buffer;
        std::vector<const_entity_ptr> m_entities;
        std::vector<interval> m_intervals;
        std::vector<collision_pair> m_collisions;

        void sort_entities();
    };
}

#endif