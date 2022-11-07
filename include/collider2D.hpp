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
        collider2D(std::vector<entity2D> &entities, std::size_t allocations = 40);

        void add(std::size_t index);
        void detect_collisions();

    private:
        struct collision_pair
        {
        public:
            collision_pair() = delete;
            collision_pair(const entity_ptr &e1, const entity_ptr &e2);
            entity_ptr e1, e2;
        };

        struct interval
        {
        public:
            enum end
            {
                LOWER,
                HIGHER
            };

            interval(const entity_ptr &e, end end_type);

            const entity_ptr &entity() const;
            float value() const;
            end type() const;

        private:
            entity_ptr m_entity;
            end m_end;
        };

        std::vector<entity2D> &m_buffer;
        std::vector<entity_ptr> m_entities;
        std::vector<interval> m_intervals;
        std::vector<collision_pair> m_collisions;

        void sort_entities();
    };
}

#endif