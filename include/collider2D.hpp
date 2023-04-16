#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "entity2D.hpp"
#include "entity2D_ptr.hpp"
#include "constraint2D.hpp"
#include "quad_tree2D.hpp"
#include "pass_key.hpp"
#include <vector>
#include <utility>

namespace ppx
{
    struct collision2D
    {
        entity2D_ptr other, incoming;
        glm::vec2 touch1, touch2, normal;
    };

    class collider2D final : public ini::saveable
    {
    public:
        enum coldet_method
        {
            BRUTE_FORCE,
            SORT_AND_SWEEP,
            QUAD_TREE
        };

        collider2D(engine_key,
                   std::vector<entity2D> *entities,
                   std::size_t allocations,
                   const glm::vec2 &min = -0.5f * glm::vec2(192.f, 128.f),
                   const glm::vec2 &max = 0.5f * glm::vec2(192.f, 128.f));

        void add_entity_intervals(const const_entity2D_ptr &e);
        void solve_and_load_collisions(std::vector<float> &stchanges);
        void update_quad_tree();
        void rebuild_quad_tree();
        void validate();

        void write(ini::output &out) const override;
        void read(ini::input &in) override;

        float stiffness() const;
        float dampening() const;

        void stiffness(float stiffness);
        void dampening(float dampening);

        bool enabled() const;
        void enabled(bool enabled);

        coldet_method coldet() const;
        void coldet(coldet_method coldet);

        const quad_tree2D &quad_tree() const;
        quad_tree2D &quad_tree();

        std::uint32_t quad_tree_build_period() const;
        void quad_tree_build_period(std::uint32_t period);

    private:
        struct interval
        {
        public:
            enum end
            {
                LOWER,
                HIGHER
            };

            interval(const const_entity2D_ptr &e, end end_type);

            const entity2D *entity() const;
            float value() const;
            end type() const;
            bool try_validate();

        private:
            const_entity2D_ptr m_entity;
            end m_end;
        };

        std::vector<entity2D> *m_entities;
        std::vector<interval> m_intervals;
        quad_tree2D m_quad_tree;
        float m_stiffness = 5000.f, m_dampening = 10.f;
        std::uint32_t m_qt_build_period = 35;
        coldet_method m_coldet_method = BRUTE_FORCE;
        bool m_enabled = true;

        void sort_intervals();
        bool collide(const entity2D &e1, const entity2D &e2, collision2D *c) const;
        void brute_force_coldet(std::vector<float> &stchanges) const;
        void sort_and_sweep_coldet(std::vector<float> &stchanges);
        void quad_tree_coldet(std::vector<float> &stchanges);

        void solve(const collision2D &c,
                   std::vector<float> &stchanges) const;
        std::array<float, 6> forces_upon_collision(const collision2D &c) const;

        bool gjk_epa(const entity2D &e1, const entity2D &e2, collision2D *c) const;

        static bool gjk(const geo::polygon &poly1, const geo::polygon &poly2, std::vector<glm::vec2> &simplex);
        static void line_case(const std::vector<glm::vec2> &simplex, glm::vec2 &dir);
        static bool triangle_case(std::vector<glm::vec2> &simplex, glm::vec2 &dir);

        static glm::vec2 epa(const geo::polygon &poly1, const geo::polygon &poly2, std::vector<glm::vec2> &simplex);
        static std::pair<glm::vec2, glm::vec2> touch_points(const geo::polygon &poly1,
                                                            const geo::polygon &poly2,
                                                            const glm::vec2 &mtv);

        collider2D(const collider2D &) = delete;
        collider2D &operator=(const collider2D &) = delete;
    };
}

#endif