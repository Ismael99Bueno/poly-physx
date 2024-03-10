#pragma once

#include "ppx/constraints/friction_constraint2D.hpp"

namespace ppx
{
class contact_constraint2D final : public constraint2D
{
  public:
    contact_constraint2D(world2D &world, const collision2D *collision, std::size_t manifold_index, float slop);

    bool recently_updated = true;

    float constraint_value() const override;
    float constraint_velocity() const override;

    void solve() override;

    void update(const collision2D *collision, float slop);

  private:
    std::size_t m_manifold_index;

    float m_restitution;
    float m_slop;
    float m_penetration;
    glm::vec2 m_mtv;
    friction_constraint2D m_friction;

    float m_init_ctr_vel = 0.f;

    float inverse_mass() const override;
    glm::vec2 direction() const override;
};
} // namespace ppx
