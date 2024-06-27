#pragma once

#include "ppx/actuators/actuator2D.hpp"
#include "ppx/common/specs2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class spring_joint2D : public actuator2D, kit::non_copyable
{
  public:
    static inline constexpr std::uint8_t ANCHORS = 2;
    using specs = specs::spring_joint2D;

    spring_joint2D(world2D &world, const specs &spc);

    glm::vec3 compute_force() const override;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    specs::properties props() const;
    void props(const specs::properties &props);

    float frequency() const;
    void frequency(float frequency);

    float damping_ratio() const;
    void damping_ratio(float damping_ratio);

    float min_length() const;
    void min_length(float min_length);

    float max_length() const;
    void max_length(float max_length);

    std::uint32_t non_linear_terms() const;
    void non_linear_terms(std::uint32_t non_linear_terms);

    float non_linear_contribution() const;
    void non_linear_contribution(float non_linear_contribution);

    static std::pair<float, float> frequency_and_ratio(float stiffness, float damping, float mass);
    static std::pair<float, float> stiffness_and_damping(float frequency, float damping_ratio, float mass);

  private:
    float m_frequency;
    float m_damping_ratio;
    float m_min_length;
    float m_max_length;
    std::uint32_t m_non_linear_terms;
    float m_non_linear_contribution;

    glm::vec2 non_linear_displacement(const glm::vec2 &displacement) const;
};

} // namespace ppx
