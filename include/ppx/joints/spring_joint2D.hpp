#pragma once

#include "ppx/joints/joint2D.hpp"
#include "ppx/common/specs2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class spring_joint2D : public joint2D, kit::non_copyable
{
  public:
    static inline constexpr std::uint8_t ANCHORS = 2;
    using specs = specs::spring_joint2D;

    spring_joint2D(world2D &world, const specs &spc);

    specs::properties props;

    glm::vec4 force() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    void solve() override;

    static std::pair<float, float> frequency_and_ratio(float stiffness, float damping, float mass);
    static std::pair<float, float> stiffness_and_damping(float frequency, float damping_ratio, float mass);

  private:
    glm::vec2 non_linear_displacement(const glm::vec2 &displacement) const;
};

} // namespace ppx
