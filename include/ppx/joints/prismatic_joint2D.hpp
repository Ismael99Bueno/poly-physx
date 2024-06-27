#pragma once

#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/interface/non_copyable.hpp"

namespace ppx
{
class prismatic_joint2D final : public pvconstraint2D<1, 1>, kit::non_copyable
{
  public:
    static inline constexpr std::uint8_t ANCHORS = 2;
    using specs = specs::prismatic_joint2D;

    prismatic_joint2D(world2D &world, const specs &spc);

    glm::vec2 constraint_position() const override;
    glm::vec2 constraint_velocity() const override;

    static glm::vec2 axis_from_angle(float radians);

    specs::properties props() const;
    void props(const specs::properties &props);

    const glm::vec2 &axis() const;
    void axis(const glm::vec2 &axis);

  private:
    glm::vec2 m_axis;

    float m_target_relangle;

    glm::vec2 direction() const override;
};
} // namespace ppx