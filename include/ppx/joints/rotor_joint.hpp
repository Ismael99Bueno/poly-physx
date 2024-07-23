#pragma once

#include "ppx/constraints/vconstraint.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class rotor_joint2D final : public vconstraint2D<0, 1>, kit::non_copyable
{
  public:
    static inline constexpr std::uint8_t ANCHORS = 0;
    using specs = specs::rotor_joint2D;

    rotor_joint2D(world2D &world, const specs &spc);

    float constraint_velocity() const override;
    void solve_velocities() override;

    specs::properties props() const;
    void props(const specs::properties &props);

    float torque() const;
    void torque(float torque);

    float correction_factor() const;
    void correction_factor(float correction_factor);

    float target_speed() const;
    void target_speed(float target_speed);

    float min_angle() const;
    void min_angle(float min_angle);

    float max_angle() const;
    void max_angle(float max_angle);

    bool spin_indefinitely() const;
    void spin_indefinitely(bool spin_indefinitely);

  private:
    void update_constraint_data() override;

    float m_torque;
    float m_correction_factor;
    float m_target_speed;
    float m_min_angle;
    float m_max_angle;
    bool m_spin_indefinitely;

    float m_correction;
    float m_relangle;
    bool m_legal_angle;
};
} // namespace ppx