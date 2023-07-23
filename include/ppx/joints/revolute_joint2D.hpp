#ifndef PPX_REVOLUTE_JOINT2D_HPP
#define PPX_REVOLUTE_JOINT2D_HPP

#include "ppx/constraints/constraint2D.hpp"
#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class revolute_joint2D : public constraint2D, public joint2D
{
  public:
    struct specs : joint2D::specs
    {
        float stiffness = 500.f, dampening = 30.f;
        static specs from_rigid_bar(const revolute_joint2D &rb);
    };
    revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, float stiffness = 500.f,
                     float dampening = 30.f);
    revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                     const glm::vec2 &anchor2, float stiffness = 500.f, float dampening = 30.f);
    revolute_joint2D(const specs &spc);

    float constraint_value() const override;
    float constraint_derivative() const override;

    bool valid() const override;
    float length() const;

#ifdef KIT_USE_YAML_CPP
    YAML::Node encode() const override;
    bool decode(const YAML::Node &node) override;
#endif

  private:
    float m_length;

    std::vector<body_gradient> constraint_gradients() const override;
    std::vector<body_gradient> constraint_derivative_gradients() const override;

    float without_anchors_constraint() const;
    float without_anchors_constraint_derivative() const;

    float with_anchors_constraint() const;
    float with_anchors_constraint_derivative() const;
};
} // namespace ppx
#endif