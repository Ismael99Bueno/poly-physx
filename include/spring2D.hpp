#ifndef SPRING2D_HPP
#define SPRING2D_HPP

#include "entity_ptr.hpp"
#include <utility>

namespace phys
{
    class spring2D
    {
    public:
        spring2D() = delete;
        spring2D(const const_entity_ptr &e1,
                 const const_entity_ptr &e2,
                 float length = 0.f);
        spring2D(const const_entity_ptr &e1,
                 const const_entity_ptr &e2,
                 const alg::vec2 &joint1,
                 const alg::vec2 &joint2,
                 float length = 0.f);

        std::tuple<alg::vec2, float, float> force() const;

        bool try_validate();

        float stiffness() const;
        float dampening() const;
        float length() const;

        void stiffness(float stiffness);
        void dampening(float dampening);
        void length(float length);

        const_entity_ptr e1() const;
        const_entity_ptr e2() const;

        void e1(const_entity_ptr e);
        void e2(const_entity_ptr e);

        alg::vec2 joint1() const;
        alg::vec2 joint2() const;

        void joint1(const alg::vec2 &joint1);
        void joint2(const alg::vec2 &joint2);

    private:
        float m_stiffness = 1.f, m_dampening = 0.f,
              m_angle1, m_angle2, m_length;
        const_entity_ptr m_e1, m_e2;
        alg::vec2 m_joint1, m_joint2;
        bool m_has_joints;

        std::tuple<alg::vec2, float, float> without_joints_force() const;
        std::tuple<alg::vec2, float, float> with_joints_force() const;
    };
}

#endif