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
        class collision_constrain : public constrain2D<2>
        {
        public:
            using constrain2D<2>::constrain2D;

        private:
            vec2 m_touch1, m_touch2;
            float constrain(const std::array<const_entity_ptr, 2> &entities) const override;
            float constrain_derivative(const std::array<const_entity_ptr, 2> &entities) const override;

            void compute_touch_points();
            std::pair<vec2, vec2> compute_touch_velocities() const;
        };

        collider2D() = delete;
        collider2D(std::vector<entity2D> &entities, std::size_t allocations = 40);

        void add(std::size_t index); // TODO: Implement remove
        const std::vector<collision_constrain> &build_collision_constrains();

    private:
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
        std::vector<collision_constrain> m_constrains;

        void sort_entities();
    };
}

#endif