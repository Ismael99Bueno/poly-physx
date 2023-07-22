#ifndef PPX_REVOLUTE_JOINT2D_HPP
#define PPX_REVOLUTE_JOINT2D_HPP

#include "ppx/constraints/constraint2D.hpp"
#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class revolute_joint2D : public constraint2D<2>, public joint2D
{
  public:
    struct specs : joint2D::specs
    {
        float stiffness = 500.f, dampening = 30.f;
        static specs from_rigid_bar(const revolute_joint2D &rb);
    };
    revolute_joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2, float stiffness = 500.f, float dampening = 30.f);
    revolute_joint2D(const entity2D::ptr &e1, const entity2D::ptr &e2, const glm::vec2 &anchor1,
                     const glm::vec2 &anchor2, float stiffness = 500.f, float dampening = 30.f);
    revolute_joint2D(const specs &spc);

    float constraint(const std::array<entity2D::const_ptr, 2> &entities) const override;
    float constraint_derivative(const std::array<entity2D::const_ptr, 2> &entities) const override;
    void bind(const entity2D::ptr &e1, const entity2D::ptr &e2) override;

    bool valid() const override;
    float length() const;

#ifdef KIT_USE_YAML_CPP
    YAML::Node encode() const override;
    bool decode(const YAML::Node &node) override;
#endif

  private:
    float m_length;

    std::array<float, 3> constraint_grad(entity2D &e) const override;
    std::array<float, 3> constraint_grad_derivative(entity2D &e) const override;

    float without_anchors_constraint(const std::array<entity2D::const_ptr, 2> &entities) const;
    float without_anchors_constraint_derivative(const std::array<entity2D::const_ptr, 2> &entities) const;

    float with_anchors_constraint(const std::array<entity2D::const_ptr, 2> &entities) const;
    float with_anchors_constraint_derivative(const std::array<entity2D::const_ptr, 2> &entities) const;
};
} // namespace ppx
#endif