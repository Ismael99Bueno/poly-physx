#pragma once

#include "ppx/constraints/constraint2D.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class contact_constraint2D : public constraint2D
{
  public:
    contact_constraint2D(const collision2D *collision, std::size_t manifold_index);

    float constraint_value() const override;
    float constraint_velocity() const override;

    bool contains(kit::uuid id) const override;
    bool valid() const override;
    void warmup() override;
    void solve() override;

  private:
    const collision2D *m_collision;

    glm::vec2 m_anchor1;
    glm::vec2 m_anchor2;

    glm::vec2 m_normal;
    std::size_t m_index;

    float m_accumulated_lambda = 0.f;

    void apply_lambda(float lambda);
    float compute_lambda() const;
};
} // namespace ppx
