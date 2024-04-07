#pragma once

#include "ppx/constraints/friction_constraint2D.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"

namespace ppx
{
class contact_constraint2D final : public pvconstraint10_2D
{
  public:
    contact_constraint2D(world2D &world, const collision2D *collision, std::size_t manifold_index);

    bool recently_updated = true;

    float constraint_position() const override;
    float constraint_velocity() const override;

    void startup() override;
    void solve_velocities() override;
    void update(const collision2D *collision, std::size_t manifold_index);

  private:
    float m_restitution;
    float m_penetration;
    float m_pntr_correction = 0.f;
    glm::vec2 m_nmtv;
    friction_constraint2D m_friction;

    bool m_has_friction;
    bool m_is_adjusting_positions = false;

    float m_init_ctr_vel = 0.f;

    void update_position_data() override;
    void warmup() override;
    float inverse_mass() const override;
    glm::vec2 direction() const override;
};
} // namespace ppx
