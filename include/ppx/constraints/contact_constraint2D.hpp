#pragma once

#include "ppx/constraints/joint_constraint2D.hpp"
#include "ppx/collision/collision2D.hpp"
#include "ppx/constraints/friction_constraint2D.hpp"

namespace ppx
{
class contact_constraint2D final : public joint_constraint2D
{
  public:
    contact_constraint2D(world2D &world, const collision2D *collision, std::size_t manifold_index, float restitution,
                         float friction, float slop);

    bool recently_updated = true;

    float constraint_value() const override;
    float constraint_velocity() const override;

    bool contains(kit::uuid id) const override;
    bool valid() const override;
    void warmup() override;
    void solve() override;

    void update(const collision2D *collision, float restitution, float friction, float slop);

  private:
    const collision2D *m_collision;

    glm::vec2 m_anchor1;
    glm::vec2 m_anchor2;

    glm::vec2 m_normal;
    std::size_t m_index;

    float m_restitution;
    float m_slop;
    friction_constraint2D m_friction;

    float m_init_ctr_vel = 0.f;
};
} // namespace ppx
