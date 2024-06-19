#pragma once

#include "ppx/collision/narrow/narrow_phase2D.hpp"

namespace ppx
{
class sat_narrow2D final : public cp_narrow_phase2D, public pp_narrow_phase2D
{
  public:
    sat_narrow2D() = default;

    virtual narrow_result2D circle_polygon(const circle &circ, const polygon &poly) const override;
    virtual narrow_result2D polygon_polygon(const polygon &poly1, const polygon &poly2) const override;
};
} // namespace ppx