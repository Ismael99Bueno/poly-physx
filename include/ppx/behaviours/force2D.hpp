#ifndef FORCE2D_HPP
#define FORCE2D_HPP
#include "ppx/internal/core.hpp"

#include "ppx/behaviours/behaviour2D.hpp"

namespace ppx
{
class force2D : public behaviour2D
{
  public:
    using behaviour2D::behaviour2D;
    virtual ~force2D() = default;

    virtual float potential_energy(const entity2D &e) const override
    {
        return 0.f;
    }
    float potential_energy() const override;

  private:
    force2D(const force2D &) = delete;
    force2D &operator=(const force2D &) = delete;
};
} // namespace ppx

#endif