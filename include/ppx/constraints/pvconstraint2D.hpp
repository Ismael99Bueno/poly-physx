#pragma once

#include "ppx/constraints/vconstraint2D.hpp"

namespace ppx
{
class pvconstraint2D;
template <typename T>
concept PVConstraint2D = requires() { requires kit::DerivedFrom<T, pvconstraint2D>; };

class pvconstraint2D : public vconstraint2D
{
  public:
    virtual ~pvconstraint2D() = default;

    virtual float constraint_position() const = 0;
    bool adjust_positions();

  protected:
    using vconstraint2D::vconstraint2D;
    float m_c = 0.f;
    bool m_needs_position_update = true;

    virtual void update_constraint_data() override;
    virtual void update_position_data();

  private:
    float compute_impulse() const override;
};
} // namespace ppx