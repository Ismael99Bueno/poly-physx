#pragma once

#include "ppx/collision/collision.hpp"
#include "ppx/constraints/vconstraint.hpp"

namespace ppx
{
class nonpen_friction2D final : public vconstraint2D<1, 0>
{
  public:
    nonpen_friction2D(world2D &world, const collision2D *collision, const glm::vec2 &normal,
                      std::size_t manifold_index);

    float max_impulse;

    float constraint_velocity() const override;
    void solve_velocities() override;
    void update(const collision2D *collision, const glm::vec2 &lanchor1, const glm::vec2 &lanchor2,
                const glm::vec2 &normal);

    static inline specs::constraint2D::properties global_props{};

  private:
    float m_friction;
    glm::vec2 m_tangent;

    glm::vec2 direction() const override;
};
} // namespace ppx