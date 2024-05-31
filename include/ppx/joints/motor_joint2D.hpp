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

    const specs::properties &props() const;
    void props(const specs::properties &props);

  private:
    void update_constraint_data() override;

    specs::properties m_props;
    glm::vec2 m_correction;
};
} // namespace ppx