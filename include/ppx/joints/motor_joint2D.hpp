#pragma once

#include "ppx/constraints/vconstraint2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class motor_joint2D final : public vconstraint2D<2, 0>, kit::non_copyable
{
  public:
    using specs = specs::motor_joint2D;

    motor_joint2D(world2D &world, const specs &spc);

    specs::properties props;

    glm::vec2 constraint_velocity() const override;
    void solve_velocities() override;
};
} // namespace ppx