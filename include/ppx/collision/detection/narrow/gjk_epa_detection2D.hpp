#pragma once

#include "ppx/collision/detection/narrow/narrow_detection2D.hpp"

namespace ppx
{
class gjk_epa_detection2D final : public cp_narrow_detection2D, public pp_narrow_detection2D
{
  public:
    gjk_epa_detection2D(float epa_threshold = 1.e-3f);

    virtual ~gjk_epa_detection2D() = default;
    virtual narrow_result circle_polygon(const circle &circ, const polygon &poly) const override;
    virtual narrow_result polygon_polygon(const polygon &poly1, const polygon &poly2) const override;

    float epa_threshold;

  private:
    narrow_result gjk_epa(const shape2D &sh1, const shape2D &sh2) const;
};
} // namespace ppx