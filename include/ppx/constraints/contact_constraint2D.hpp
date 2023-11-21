#ifndef PPX_CONTACT_CONSTRAINT_HPP
#define PPX_CONTACT_CONSTRAINT_HPP

#include "ppx/constraints/constraint2D.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
class contact_constraint2D : public constraint2D
{
  public:
    contact_constraint2D(const collision2D *collision);

    float constraint_value() const override;
    float constraint_velocity() const override;
    float constraint_acceleration() const override;

    bool contains(kit::uuid id) const override;

  private:
    const collision2D *m_collision;

    glm::vec2 m_anchor1;
    glm::vec2 m_anchor2;
    glm::vec2 m_normal;

    float m_accumulated_lambda = 0.f;

    bool valid() const override;

    void apply_lambda(float lambda);
    float compute_lambda() const;

    void warmup() override;
    void solve() override;
};
} // namespace ppx

#endif