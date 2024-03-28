#pragma once

#include "ppx/constraints/vconstraint2D.hpp"

namespace ppx
{
class pvconstraint2D : public vconstraint2D
{
  public:
    virtual ~pvconstraint2D() = default;

    virtual float constraint_position() const = 0;
    virtual void startup() override;
    bool adjust_positions();

  protected:
    using vconstraint2D::vconstraint2D;
    float m_c = 0.f;

  private:
    float compute_velocity_lambda() const override;
};
} // namespace ppx