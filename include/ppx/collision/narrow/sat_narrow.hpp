#pragma once

#include "ppx/collision/narrow/narrow_phase.hpp"

namespace ppx
{
class sat_narrow2D final : public narrow_phase2D
{
  public:
    using narrow_phase2D::narrow_phase2D;

    const char *name() const override;

    result circle_polygon(const circle &circ, const polygon &poly) const override;
    result polygon_polygon(const polygon &poly1, const polygon &poly2) const override;
};
} // namespace ppx