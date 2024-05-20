#pragma once

#include "ppx/collision/resolution/collision_resolution2D.hpp"

namespace ppx
{
class spring_driven_resolution2D : public collision_resolution2D
{
  public:
    spring_driven_resolution2D(world2D &world, float rigidity = 2000.f, float normal_damping = 8.5f,
                               float tangen_damping = 8.5f);

    float rigidity;
    float normal_damping;
    float tangent_damping;

  private:
    void solve_collisions(const collision_detection2D::collision_map &collisions) override;

    std::tuple<glm::vec2, float, float> compute_collision_forces(const collision2D &colis,
                                                                 std::size_t manifold_index) const;
    void solve_and_apply_collision_forces(const collision2D &colis) const;
};
} // namespace ppx
