#pragma once

#include "ppx/collision/collision2D.hpp"
#include "ppx/constraints/vconstraint2D.hpp"

namespace ppx
{
class nonpen_friction2D final : public vconstraint2D<1, 0>
{
  public:
    nonpen_friction2D(world2D &world, const collision2D *collision, const glm::vec2 &nmtv, std::size_t manifold_index);

    float max_impulse;

    float constraint_velocity() const override;
    void solve_velocities() override;
    void update(const collision2D *collision, const glm::vec2 &lanchor1, const glm::vec2 &nmtv);

  private:
    float m_friction;
    glm::vec2 m_tangent;

    glm::vec2 direction() const override;
};
} // namespace ppx