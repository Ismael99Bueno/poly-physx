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
        static specs from_revolute_joint(const revolute_joint2D &rj);
    };
    revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2);
    revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                     const glm::vec2 &anchor2);
    revolute_joint2D(const specs &spc);

    float constraint_value() const override;
    float constraint_derivative() const override;

    bool valid() const override;
    bool contains(kit::uuid id) const override;

    float length() const;

#ifdef KIT_USE_YAML_CPP
    YAML::Node encode() const override;
    bool decode(const YAML::Node &node) override;
#endif

  private:
    float m_length;
    float m_accumulated_impulse1 = 0.f;
    float m_accumulated_impulse2 = 0.f;

    float without_anchors_constraint() const;
    float without_anchors_constraint_derivative() const;

    float with_anchors_constraint() const;
    float with_anchors_constraint_derivative() const;

    std::pair<float, float> compute_impulses() const;

    bool any_kinematic() const override;
    void warmup() override;
    void solve() override;
    void finalize(std::vector<float> &state_derivative) override;
};
} // namespace ppx
#endif