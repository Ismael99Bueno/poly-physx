#pragma once

#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class world2D;
class constraint2D : public joint2D, public worldref2D
{
  public:
    virtual ~constraint2D() = default;

    virtual float constraint_value() const = 0;
    virtual float constraint_velocity() const = 0;

  protected:
    constraint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &ganchor1,
                 const glm::vec2 &ganchor2, bool m_allow_baumgarte = true);
    constraint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &ganchor,
                 bool m_allow_baumgarte = true);
    float m_cumlambda = 0.f;

    float compute_lambda(const std::tuple<glm::vec2, glm::vec2, glm::vec2> &dir_and_offsets) const;
    void apply_lambda(float lambda, const std::tuple<glm::vec2, glm::vec2, glm::vec2> &dir_and_offsets);

    void solve_clamped(float min, float max);
    void solve_unclamped();

  private:
    void warmup();
    virtual void solve() = 0;
    bool m_allow_baumgarte;
};
} // namespace ppx
