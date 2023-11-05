#ifndef PPX_SPRING_SOLVER2D_HPP
#define PPX_SPRING_SOLVER2D_HPP

#include "ppx/collision/collision_solver2D.hpp"

namespace ppx
{
class spring_solver2D : public collision_solver2D
{
  public:
    void solve(const std::vector<collision2D> &collisions) const override;
    void solve_and_apply_collision_forces(const collision2D &colis) const;

    static float rigidity_coeff();
    static float restitution_coeff();
    static float friction_coeff();

    static void rigidity_coeff(float rigidity);
    static void restitution_coeff(float restitution);
    static void friction_coeff(float friction);

    static float rigidity();
    static float restitution();
    static float friction();

  private:
    static inline float s_rigidity_coeff = 0.985f;
    static inline float s_restitution_coeff = 0.4f;
    static inline float s_friction_coeff = 0.6f;

    static float s_rigidity;
    static float s_restitution;
    static float s_friction;
};
} // namespace ppx

#endif