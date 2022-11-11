#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "entity2D.hpp"
#include "entity_ptr.hpp"
#include "constrain2D.hpp"
#include <vector>
#include <utility>

#define VAR_PER_ENTITY 6

namespace physics
{
    class collider2D
    {
    public:
        collider2D(float stiffness = 150.f,
                   float dampening = 0.f,
                   std::size_t allocations = 40);

        void add_entity(const const_entity_ptr &e); // TODO: Implement remove
        void solve_and_load_collisions(std::vector<float> &stchanges);

    private:
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

        struct collision_pair
        {
            collision_pair(const const_entity_ptr &e1, const const_entity_ptr &e2);
            const_entity_ptr e1, e2;
        };

        std::vector<const_entity_ptr> m_entities;
        std::vector<interval> m_intervals;
        float m_stiffness, m_dampening;

        void sort_intervals();

        std::vector<collision_pair> detect_collisions() const;

        void load_collisions(const std::vector<collision_pair> &collisions,
                             std::vector<float> &stchanges) const;
        std::pair<vec2, vec2> touch_points(const const_entity_ptr &e1, const const_entity_ptr &e2) const;
        std::array<float, VAR_PER_ENTITY> state_changes_upon_collision(const const_entity_ptr &e1,
                                                                       const const_entity_ptr &e2,
                                                                       const vec2 &touch1,
                                                                       const vec2 &touch2) const;
    };
}

#endif