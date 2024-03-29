#pragma once

#include "geo/shapes2D/polygon.hpp"
#include "geo/shapes2D/circle.hpp"

#ifndef PPX_MAX_VERTICES
#define PPX_MAX_VERTICES 8
#endif

namespace ppx
{
using shape2D = geo::shape2D;
using circle = geo::circle;
using aabb2D = geo::aabb2D;
using polygon = geo::polygon<PPX_MAX_VERTICES>;
using transform2D = geo::transform2D;
} // namespace ppx