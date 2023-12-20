#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"

namespace ppx
{
class spring_driven_resolution2D : public collision_resolution2D
{
  public:
    static inline float rigidity = 2000.f;
    static inline float normal_damping = 5.f;
    static inline float tangent_damping = 5.f;

  private:
    void solve(const std::vector<collision2D> &collisions) const override;

    std::tuple<glm::vec2, float, float> compute_collision_forces(const collision2D &colis,
                                                                 std::size_t manifold_index) const;
    void solve_and_apply_collision_forces(const collision2D &colis) const;
};
} // namespace ppx
