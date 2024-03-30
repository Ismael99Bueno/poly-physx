#pragma once

#include "ppx/collision/collision2D.hpp"
#include "ppx/constraints/vconstraint2D.hpp"

namespace ppx
{
class friction_constraint2D final : public vconstraint2D
{
  public:
    friction_constraint2D(world2D &world, const collision2D *collision, std::size_t manifold_index);

    float max_lambda;

    float constraint_velocity() const override;
    void solve() override;
    void update(const collision2D *collision, const glm::vec2 &lanchor1, const glm::vec2 &lanchor2);

  private:
    float m_friction;
    glm::vec2 m_mtv;

    float inverse_mass() const override;
    glm::vec2 direction() const override;
};
} // namespace ppx