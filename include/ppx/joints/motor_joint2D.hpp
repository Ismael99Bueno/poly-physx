#pragma once

#include "ppx/constraints/vconstraint2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class motor_joint2D final : public vconstraint2D<2, 0>, kit::non_copyable
{
  public:
    static inline constexpr std::uint8_t ANCHORS = 0;
    using specs = specs::motor_joint2D;

    motor_joint2D(world2D &world, const specs &spc);

    glm::vec2 constraint_velocity() const override;
    void solve_velocities() override;

    specs::properties props() const;
    void props(const specs::properties &props);

    float force() const;
    void force(float force);

    float correction_factor() const;
    void correction_factor(float correction_factor);

    float target_speed() const;
    void target_speed(float target_speed);

    const glm::vec2 &target_offset() const;
    void target_offset(const glm::vec2 &target_offset);

  private:
    void update_constraint_data() override;

    float m_force;
    float m_correction_factor;
    float m_target_speed;
    glm::vec2 m_target_offset;

    glm::vec2 m_correction;
};
} // namespace ppx