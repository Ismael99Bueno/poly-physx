#pragma once

#include "ppx/common/alias.hpp"
#include "ppx/collision/collision2D.hpp"
#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
struct narrow_result2D
{
    bool intersects = false;
    glm::vec2 mtv{0.f};
    manifold2D manifold;
    operator bool() const
    {
        return intersects;
    }
};

class cp_narrow_phase2D
{
  public:
    virtual ~cp_narrow_phase2D() = default;
    virtual narrow_result2D circle_polygon(const circle &circ, const polygon &poly) const = 0;
};

class pp_narrow_phase2D
{
  public:
    virtual ~pp_narrow_phase2D() = default;
    virtual narrow_result2D polygon_polygon(const polygon &poly1, const polygon &poly2) const = 0;
};
} // namespace ppx