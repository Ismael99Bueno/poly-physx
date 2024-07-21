#pragma once

#include "ppx/behaviours/behaviour2D.hpp"

namespace ppx
{
class force2D : public behaviour2D
{
  public:
    using behaviour2D::behaviour2D;
    virtual ~force2D() = default;

    virtual float potential_energy(const state2D &state) const override
    {
        return 0.f;
    }
    float potential_energy() const override final;
};
} // namespace ppx
