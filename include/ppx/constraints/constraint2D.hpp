#pragma once

#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class constraint2D : public joint2D
{
  public:
    virtual ~constraint2D() = default;

    virtual float constraint_value() const = 0;
    virtual float constraint_velocity() const = 0;

    virtual void startup();
    virtual void warmup();

  protected:
    constraint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor1, const glm::vec2 &ganchor2,
                 bool baumgarte_correction = true);
    constraint2D(world2D &world, body2D *body1, body2D *body2, const glm::vec2 &ganchor,
                 bool baumgarte_correction = true);
    float m_cumlambda = 0.f;

    glm::vec2 m_ganchor1;
    glm::vec2 m_ganchor2;

    glm::vec2 m_offset1;
    glm::vec2 m_offset2;

    glm::vec2 m_dir;
    float m_inv_mass;

    void solve_clamped(float min, float max);
    void solve_unclamped();

  private:
    bool m_allow_baumgarte;

    virtual float inverse_mass() const = 0;
    virtual glm::vec2 direction() const = 0;

    float compute_lambda() const;
    void apply_lambda(float lambda);

    friend class constraint_driven_resolution2D;
};
} // namespace ppx
