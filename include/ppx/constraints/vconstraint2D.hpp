#pragma once

#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class vconstraint2D : public joint2D
{
  public:
    virtual ~vconstraint2D() = default;

    virtual float constraint_velocity() const = 0;

    virtual void startup();
    virtual void warmup();
    void reset_impulse();

  protected:
    vconstraint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2);
    vconstraint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor);
    float m_cumlambda = 0.f;

    glm::vec2 m_ganchor1;
    glm::vec2 m_ganchor2;

    glm::vec2 m_offset1;
    glm::vec2 m_offset2;

    glm::vec2 m_dir;
    float m_inv_mass;

    void solve_clamped(float min, float max);
    void solve_unclamped();

    virtual float compute_velocity_lambda() const;

  private:
    virtual float inverse_mass() const = 0;
    virtual glm::vec2 direction() const = 0;

    void apply_velocity_lambda(float lambda);
};

} // namespace ppx
