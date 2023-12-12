#ifndef PPX_SPRING_SOLVER2D_HPP
#define PPX_SPRING_SOLVER2D_HPP

#include "ppx/collision/solvers/collision_solver2D.hpp"

namespace ppx
{
class spring_driven_solver2D : public collision_solver2D
{
  public:
    static inline float rigidity_coeff = 0.985f;

    static float rigidity();
    static float restitution();
    static float friction();

  private:
    void solve(const std::vector<collision2D> &collisions) const override;

    std::tuple<glm::vec2, float, float> compute_collision_forces(const collision2D &colis,
                                                                 std::size_t manifold_index) const;
    void solve_and_apply_collision_forces(const collision2D &colis) const;
};
} // namespace ppx

#endif