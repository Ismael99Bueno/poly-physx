#pragma once

#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class weld_joint2D final : public pvconstraint2D<2, 1>, kit::non_copyable
{
  public:
    using specs = specs::weld_joint2D;

    weld_joint2D(world2D &world, const specs &spc);

    glm::vec3 constraint_position() const override;
    glm::vec3 constraint_velocity() const override;

  private:
    float m_target_relangle;
};
} // namespace ppx