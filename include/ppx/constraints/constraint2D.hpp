#pragma once

#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class constraint2D;
template <typename T>
concept IConstraint2D = kit::DerivedFrom<T, constraint2D>;

class constraint2D : virtual public joint2D
{
  public:
    using joint2D::joint2D;

    virtual void startup() = 0;
    virtual void solve_velocities() = 0;

    virtual bool solve_positions();
    bool is_constraint() const override;
};
} // namespace ppx