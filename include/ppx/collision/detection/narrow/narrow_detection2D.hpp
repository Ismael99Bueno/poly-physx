#pragma once

#include "ppx/common/shapes2D.hpp"

namespace ppx
{
struct narrow_result
{
    bool valid = false;
    glm::vec2 mtv;
};

class cp_narrow_detection2D
{
  public:
    virtual ~cp_narrow_detection2D() = default;
    virtual narrow_result circle_polygon(const circle &circ, const polygon &poly) const = 0;
};

class pp_narrow_detection2D
{
  public:
    virtual ~pp_narrow_detection2D() = default;
    virtual narrow_result polygon_polygon(const polygon &poly1, const polygon &poly2) const = 0;
};
} // namespace ppx