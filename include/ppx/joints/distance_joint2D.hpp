#pragma once

#include "ppx/constraints/joint_constraint2D.hpp"
#include "ppx/joints/joint_proxy2D.hpp"

namespace ppx
{
class distance_joint2D final : public joint_constraint2D
{
  public:
    struct specs
    {
        joint_proxy2D::specs joint;
        static specs from_distance_joint(const distance_joint2D &dj);
    };
    distance_joint2D();
    distance_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1 = glm::vec2(0.f),
                     const glm::vec2 &anchor2 = glm::vec2(0.f));
    distance_joint2D(const specs &spc);

    joint_proxy2D joint;
    float length = 0.f;

    float constraint_value() const override;
    float constraint_velocity() const override;

    bool valid() const override;
    bool contains(kit::uuid id) const override;

#ifdef KIT_USE_YAML_CPP
    YAML::Node encode() const override;
    bool decode(const YAML::Node &node) override;
#endif

  private:
    void warmup() override;
    void solve() override;
};
} // namespace ppx
