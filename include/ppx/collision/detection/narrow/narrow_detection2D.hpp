#pragma once

#include "ppx/common/alias.hpp"
#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
using narrow_result2D = geo::sat_result2D;

class cp_narrow_detection2D
{
  public:
    virtual ~cp_narrow_detection2D() = default;
    virtual narrow_result2D circle_polygon(const circle &circ, const polygon &poly) const = 0;
};

class pp_narrow_detection2D
{
  public:
    virtual ~pp_narrow_detection2D() = default;
    virtual narrow_result2D polygon_polygon(const polygon &poly1, const polygon &poly2) const = 0;
};
} // namespace ppx