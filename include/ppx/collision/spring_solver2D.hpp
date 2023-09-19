#ifndef PPX_SPRING_SOLVER2D_HPP
#define PPX_SPRING_SOLVER2D_HPP

#include "ppx/collision/collision_solver2D.hpp"

namespace ppx
{
class spring_solver2D : public collision_solver2D
{
  public:
    float stiffness = 5000.f;
    float dampening = 10.f;

    void solve(const std::vector<collision2D> &collisions, std::vector<float> &state_derivative) const override;
    void solve(const collision2D &colis, std::vector<float> &state_derivative) const;

  private:
    std::array<float, 6> forces_upon_collision(const collision2D &colis) const;
};
} // namespace ppx

#endif