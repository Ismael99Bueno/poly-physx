#pragma once

#include "ppx/collision/narrow/narrow_phase2D.hpp"

namespace ppx
{
class gjk_epa_narrow2D final : public cp_narrow_phase2D, public pp_narrow_phase2D
{
  public:
    gjk_epa_narrow2D(float epa_threshold = 1.e-3f);

    const char *name() const override;

    narrow_result2D circle_polygon(const circle &circ, const polygon &poly) const override;
    narrow_result2D polygon_polygon(const polygon &poly1, const polygon &poly2) const override;

    float epa_threshold;

  private:
    narrow_result2D gjk_epa(const shape2D &sh1, const shape2D &sh2) const;
};
} // namespace ppx