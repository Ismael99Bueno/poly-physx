#pragma once

#include "ppx/constraints/pvconstraint.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class revolute_joint2D final : public pvconstraint2D<2, 0>, kit::non_copyable
{
  public:
    static inline constexpr std::uint8_t ANCHORS = 1;
    using specs = specs::revolute_joint2D;

    revolute_joint2D(world2D &world, const specs &spc);

    glm::vec2 constraint_position() const override;
    glm::vec2 constraint_velocity() const override;

    specs::properties props() const;
    void props(const specs::properties &props);
};
} // namespace ppx