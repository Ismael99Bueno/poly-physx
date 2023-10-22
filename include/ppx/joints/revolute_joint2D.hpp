#ifndef PPX_REVOLUTE_JOINT2D_HPP
#define PPX_REVOLUTE_JOINT2D_HPP

#include "ppx/constraints/constraint2D.hpp"
#include "ppx/joints/joint_proxy2D.hpp"

namespace ppx
{
class revolute_joint2D : public constraint2D
{
  public:
    struct specs
    {
        joint_proxy2D::specs joint;
        static specs from_revolute_joint(const revolute_joint2D &rj);
    };
    revolute_joint2D();
    revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1 = glm::vec2(0.f),
                     const glm::vec2 &anchor2 = glm::vec2(0.f));
    revolute_joint2D(const specs &spc);

    joint_proxy2D joint;
    float length = 0.f;

    float constraint_value() const override;
    float constraint_derivative() const override;

    bool valid() const override;
    bool contains(kit::uuid id) const override;

#ifdef KIT_USE_YAML_CPP
    YAML::Node encode() const override;
    bool decode(const YAML::Node &node) override;
#endif

  private:
    float m_accumulated_impulse1 = 0.f;
    float m_accumulated_impulse2 = 0.f;

    std::tuple<glm::vec2, glm::vec2, glm::vec2> compute_anchors_and_direction() const;
    std::pair<float, float> compute_impulses() const;
    void apply_impulses(float imp1, float imp2);

    void warmup() override;
    void solve() override;

    friend class world2D;
};
} // namespace ppx
#endif