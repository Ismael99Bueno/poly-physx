#pragma once

#include "ppx/constraints/vconstraint2D.hpp"

namespace ppx
{
class pvconstraint2D : public vconstraint2D
{
  public:
    virtual ~pvconstraint2D() = default;

    virtual float constraint_position() const = 0;
    virtual bool adjust_positions() = 0;

    virtual void startup() override;

  protected:
    pvconstraint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2);
    pvconstraint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor);
    float m_c;

    void adjust_clamped(float min, float max);
    void adjust_unclamped();

  private:
    float compute_velocity_lambda() const override;
    float compute_position_lambda() const;
    void apply_position_lambda(float lambda);
};
} // namespace ppx