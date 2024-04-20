#pragma once

#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class distance_joint2D final : public pvconstraint2D<1, 0>, kit::non_copyable
{
  public:
    using specs = specs::distance_joint2D;

    distance_joint2D(world2D &world, const specs &spc);

    specs::properties props;

    float constraint_position() const override;
    float constraint_velocity() const override;

    void solve_velocities() override;

  private:
    void update_constraint_data() override;
    glm::vec2 direction() const override;

    float m_length;
    bool m_legal_length;
};
} // namespace ppx
