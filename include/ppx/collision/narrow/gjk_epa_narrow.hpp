#pragma once

#include "ppx/collision/narrow/narrow_phase.hpp"

namespace ppx
{
class gjk_epa_narrow2D final : public narrow_phase2D
{
  public:
    gjk_epa_narrow2D(world2D &world, float epa_threshold = 1.e-3f);

    const char *name() const override;

    result circle_polygon(const circle &circ, const polygon &poly) const override;
    result polygon_polygon(const polygon &poly1, const polygon &poly2) const override;

    float epa_threshold;

  private:
    result gjk_epa(const shape2D &sh1, const shape2D &sh2) const;
};
} // namespace ppx