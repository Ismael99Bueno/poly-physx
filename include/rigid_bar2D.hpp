#ifndef RIGID_BAR2D_HPP
#define RIGID_BAR2D_HPP

#include "constrain2D.hpp"

namespace phys
{
    class rigid_bar2D : public constrain2D<2>
    {
    public:
        rigid_bar2D() = delete;
        rigid_bar2D(const entity_ptr &e1,
                    const entity_ptr &e2,
                    float length = 10.f);
        rigid_bar2D(const entity_ptr &e1,
                    const entity_ptr &e2,
                    const alg::vec2 &joint1,
                    const alg::vec2 &joint2,
                    float length = 10.f);

        float constrain(const std::array<const_entity_ptr, 2> &entities) const override;
        float constrain_derivative(const std::array<const_entity_ptr, 2> &entities) const override;

        float length() const;
        void length(float length);

        const_entity_ptr e1() const;
        const_entity_ptr e2() const;

        const alg::vec2 &joint1() const;
        const alg::vec2 &joint2() const;

    private:
        float m_length;
        alg::vec2 m_joint1, m_joint2;
        bool m_has_joints;

        float without_joints_constrain(const std::array<const_entity_ptr, 2> &entities) const;
        float without_joints_constrain_derivative(const std::array<const_entity_ptr, 2> &entities) const;

        float with_joints_constrain(const std::array<const_entity_ptr, 2> &entities) const;
        float with_joints_constrain_derivative(const std::array<const_entity_ptr, 2> &entities) const;
    };
}

#endif