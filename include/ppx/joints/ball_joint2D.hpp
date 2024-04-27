#pragma once

#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class ball_joint2D final : public pvconstraint2D<0, 1>, kit::non_copyable
{
  public:
    static inline constexpr std::uint8_t ANCHORS = 0;
    using specs = specs::ball_joint2D;

    ball_joint2D(world2D &world, const specs &spc);

    specs::properties props;

    float constraint_position() const override;
    float constraint_velocity() const override;

    bool solve_positions() override;
    void solve_velocities() override;

  private:
    float m_relangle;
    bool m_legal_angle;

    void update_constraint_data() override;
};
} // namespace ppx