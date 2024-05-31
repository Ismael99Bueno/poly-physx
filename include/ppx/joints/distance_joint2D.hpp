#pragma once

#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class distance_joint2D final : public pvconstraint2D<1, 0>, kit::non_copyable
{
  public:
    static inline constexpr std::uint8_t ANCHORS = 2;
    using specs = specs::distance_joint2D;

    distance_joint2D(world2D &world, const specs &spc);

    float constraint_position() const override;
    float constraint_velocity() const override;

    bool solve_positions() override;
    void solve_velocities() override;

    const specs::properties &props() const;
    void props(const specs::properties &props);

  private:
    void update_constraint_data() override;
    glm::vec2 direction() const override;

    specs::properties m_props;

    float m_length;
    bool m_legal_length;
};
} // namespace ppx
