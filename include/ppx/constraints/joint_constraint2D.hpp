#pragma once

#include "ppx/constraints/constraint2D.hpp"

namespace ppx
{
class joint_constraint2D : public constraint2D
{
  public:
    joint_constraint2D(const char *name, bool allow_position_corrections = true);

  protected:
    float compute_lambda(body2D &body1, body2D &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
                         const glm::vec2 &dir) const;

    void apply_warmup(body2D &body1, body2D &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
                      const glm::vec2 &dir);
    void apply_lambda(float lambda, body2D &body1, body2D &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
                      const glm::vec2 &dir) const;

    void solve_clamped(body2D &body1, body2D &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
                       const glm::vec2 &dir, float min, float max);
    void solve_unclamped(body2D &body1, body2D &body2, const glm::vec2 &anchor1, const glm::vec2 &anchor2,
                         const glm::vec2 &dir);

  private:
    bool m_allow_position_corrections;
};
} // namespace ppx