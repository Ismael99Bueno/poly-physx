#ifndef COLLIDER_HPP
#define COLLIDER_HPP
#include "ppx/core.hpp"

#include "ppx/entity2D.hpp"
#include "ppx/entity2D_ptr.hpp"
#include "ppx/constraint2D.hpp"
#include "ppx/quad_tree2D.hpp"
#include <vector>
#include <utility>

namespace ppx
{
    struct collision2D
    {
        entity2D_ptr current, incoming;
        glm::vec2 touch1{0.f}, touch2{0.f}, normal{0.f};
    };

    class collider2D final
    {
    public:
        enum detection_method
        {
            BRUTE_FORCE = 0,
            SORT_AND_SWEEP = 1,
            QUAD_TREE = 2
        };

        collider2D(std::vector<entity2D> *entities,
                   std::size_t allocations,
                   const glm::vec2 &min = -0.5f * glm::vec2(192.f, 128.f),
                   const glm::vec2 &max = 0.5f * glm::vec2(192.f, 128.f));

        void add_entity_intervals(const const_entity2D_ptr &e);
        void solve_and_load_collisions(std::vector<float> &stchanges);
        void update_quad_tree();
        void rebuild_quad_tree();
        void validate();

        float stiffness() const;
        float dampening() const;

        void stiffness(float stiffness);
        void dampening(float dampening);

        bool enabled() const;
        void enabled(bool enabled);

        detection_method detection() const;
        void detection(detection_method coldet);

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
            bool validate();

        private:
            const_entity2D_ptr m_entity;
            end m_end;
        };

        std::vector<entity2D> *m_entities;
        std::vector<interval> m_intervals;
        quad_tree2D m_quad_tree;
        float m_stiffness = 5000.f, m_dampening = 10.f;
        std::uint32_t m_qt_build_period = 35;
        detection_method m_coldet_method = QUAD_TREE;
        bool m_enabled = true;

        void sort_intervals();
        bool collide(const entity2D &e1, const entity2D &e2, collision2D *c) const;
        void try_enter_or_stay_callback(const entity2D &e1, const entity2D &e2, const collision2D &c) const;
        void try_exit_callback(const entity2D &e1, const entity2D &e2) const;

        void brute_force_coldet(std::vector<float> &stchanges) const;
        void sort_and_sweep_coldet(std::vector<float> &stchanges);
        void quad_tree_coldet(std::vector<float> &stchanges);

        void solve(const collision2D &c,
                   std::vector<float> &stchanges) const;
        std::array<float, 6> forces_upon_collision(const collision2D &c) const;

        collider2D(const collider2D &) = delete;
        collider2D &operator=(const collider2D &) = delete;
    };

#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const collider2D &cld);
#endif
}

#ifdef HAS_YAML_CPP
namespace YAML
{
    template <>
    struct convert<ppx::collider2D>
    {
        static Node encode(const ppx::collider2D &cld);
        static bool decode(const Node &node, ppx::collider2D &cld);
    };
}
#endif

#endif