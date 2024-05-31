#pragma once

#include "ppx/constraints/vconstraint2D.hpp"
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

    const specs::properties &props() const;
    void props(const specs::properties &props);

  private:
    void update_constraint_data() override;

    specs::properties m_props;
    float m_correction;
    float m_relangle;
    bool m_legal_angle;
};
} // namespace ppx