#pragma once

#include "ppx/constraints/constraint2D.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class friction_constraint2D final : public constraint2D
{
  public:
    friction_constraint2D(const collision2D *collision, std::size_t manifold_index, float friction);

    float max_lambda;

    float constraint_value() const override;
    float constraint_velocity() const override;

    bool contains(kit::uuid id) const override;
    bool valid() const override;
    void warmup() override;
    void solve() override;

    void update(const collision2D *collision, const glm::vec2 &normal, const glm::vec2 &anchor1,
                const glm::vec2 &anchor2, float friction);

  private:
    const collision2D *m_collision;

    glm::vec2 m_anchor1;
    glm::vec2 m_anchor2;

    glm::vec2 m_tangent;

    float m_friction;
    float m_accumulated_lambda;

    void apply_lambda(float lambda);
    float compute_lambda() const;
};
} // namespace ppx