#pragma once

#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class revolute_joint2D final : public pvconstraint2D<2, 0>, kit::non_copyable
{
  public:
    using specs = specs::revolute_joint2D;

    revolute_joint2D(world2D &world, const specs &spc);

    glm::vec2 constraint_position() const override;
    glm::vec2 constraint_velocity() const override;
};
} // namespace ppx