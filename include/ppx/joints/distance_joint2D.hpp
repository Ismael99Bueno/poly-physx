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

    float min_distance;
    float max_distance;

    float constraint_position() const override;
    float constraint_velocity() const override;

    void startup() override;
    void solve_velocities() override;

  private:
    glm::vec2 direction() const override;

    bool legal_length() const;

    float m_length;
};
} // namespace ppx
