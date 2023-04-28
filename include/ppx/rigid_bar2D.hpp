#ifndef RIGID_BAR2D_HPP
#define RIGID_BAR2D_HPP

#include "ppx/constraint2D.hpp"

namespace ppx
{
    class rigid_bar2D : public constraint2D<2>, ini::saveable
    {
    public:
        rigid_bar2D(const entity2D_ptr &e1,
                    const entity2D_ptr &e2,
                    float stiffness = 1.f,
                    float dampening = 0.f);
        rigid_bar2D(const entity2D_ptr &e1,
                    const entity2D_ptr &e2,
                    const glm::vec2 &joint1,
                    const glm::vec2 &joint2,
                    float stiffness = 1.f,
                    float dampening = 0.f);

        float constraint(const std::array<const_entity2D_ptr, 2> &entities) const override;
        float constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const override;

        float length() const;
        void length(float length);

        void write(ini::output &out) const override;
        void read(ini::input &in) override;

        const entity2D_ptr &e1() const;
        const entity2D_ptr &e2() const;

        glm::vec2 joint1() const;
        glm::vec2 joint2() const;

        void joint1(const glm::vec2 &joint1);
        void joint2(const glm::vec2 &joint2);

        bool has_joints() const;
        bool try_validate() override;

    private:
        entity2D_ptr m_e1, m_e2;
        float m_length, m_angle1, m_angle2;
        glm::vec2 m_joint1{0.f}, m_joint2{0.f};
        bool m_has_joints;

        std::array<float, 3> constraint_grad(entity2D &e) const override;
        std::array<float, 3> constraint_grad_derivative(entity2D &e) const override;

        float without_joints_constraint(const std::array<const_entity2D_ptr, 2> &entities) const;
        float without_joints_constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const;

        float with_joints_constraint(const std::array<const_entity2D_ptr, 2> &entities) const;
        float with_joints_constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const;
    };
}

#endif