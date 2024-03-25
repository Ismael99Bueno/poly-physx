#pragma once

#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class spring2D : public joint2D
{
  public:
    using specs = specs::spring2D;

    spring2D(world2D &world, const specs &spc);

    float frequency;
    float damping_ratio;
    float length;

    std::uint32_t non_linear_terms;
    float non_linear_contribution;

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
