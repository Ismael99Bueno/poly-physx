#pragma once

#include "ppx/constraints/pvconstraint.hpp"
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

    specs::properties props() const;
    void props(const specs::properties &props);

    float min_distance() const;
    void min_distance(float min_distance);

    float max_distance() const;
    void max_distance(float max_distance);

  private:
    void update_constraint_data() override;
    glm::vec2 direction() const override;

    float m_min_distance;
    float m_max_distance;

    float m_length;
    bool m_legal_length;
};
} // namespace ppx
