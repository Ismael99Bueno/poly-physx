#ifndef RIGID_BAR2D_HPP
#define RIGID_BAR2D_HPP

#include "ppx/constraint2D.hpp"

namespace ppx
{
    class rigid_bar2D : public constraint2D<2>
    {
    public:
        rigid_bar2D(const entity2D_ptr &e1,
                    const entity2D_ptr &e2,
                    float stiffness = 1.f,
                    float dampening = 0.f);
        rigid_bar2D(const entity2D_ptr &e1,
                    const entity2D_ptr &e2,
                    const glm::vec2 &anchor1,
                    const glm::vec2 &anchor2,
                    float stiffness = 1.f,
                    float dampening = 0.f);

        float constraint(const std::array<const_entity2D_ptr, 2> &entities) const override;
        float constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const override;

        float length() const;
        void length(float length);

        const entity2D_ptr &e1() const;
        const entity2D_ptr &e2() const;

        glm::vec2 anchor1() const;
        glm::vec2 anchor2() const;

        void anchor1(const glm::vec2 &anchor1);
        void anchor2(const glm::vec2 &anchor2);

        bool has_anchors() const;
        bool try_validate() override;

    private:
        entity2D_ptr m_e1, m_e2;
        float m_length, m_angle1, m_angle2;
        glm::vec2 m_anchor1{0.f}, m_anchor2{0.f};
        bool m_has_anchors;

        std::array<float, 3> constraint_grad(entity2D &e) const override;
        std::array<float, 3> constraint_grad_derivative(entity2D &e) const override;

        float without_anchors_constraint(const std::array<const_entity2D_ptr, 2> &entities) const;
        float without_anchors_constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const;

        float with_anchors_constraint(const std::array<const_entity2D_ptr, 2> &entities) const;
        float with_anchors_constraint_derivative(const std::array<const_entity2D_ptr, 2> &entities) const;
    };
#ifdef HAS_YAML_CPP
    YAML::Emitter &operator<<(YAML::Emitter &out, const rigid_bar2D &rb);
#endif
}
#ifdef HAS_YAML_CPP
namespace YAML
{
    template <>
    struct convert<ppx::rigid_bar2D>
    {
        static Node encode(const ppx::rigid_bar2D &rb);
        static bool decode(const Node &node, ppx::rigid_bar2D &rb);
    };
}
#endif
#endif