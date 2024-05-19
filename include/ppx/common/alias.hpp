#pragma once

#include "geo/shapes2D/polygon.hpp"
#include "geo/shapes2D/circle.hpp"
#include "geo/algorithm/ray2D.hpp"
#ifdef PPX_ENABLE_BLOCK_ALLOCATOR
#include "kit/memory/allocator/block_allocator.hpp"
#else
#include "kit/memory/allocator/vanilla_allocator.hpp"
#endif
#include "kit/container/quad_tree.hpp"

#ifndef PPX_MAX_VERTICES
#define PPX_MAX_VERTICES 8
#endif

namespace ppx
{
class collider2D;
using shape2D = geo::shape2D;
using circle = geo::circle;
using aabb2D = geo::aabb2D;
using polygon = geo::polygon<PPX_MAX_VERTICES>;
using transform2D = geo::transform2D;
using ray2D = geo::ray2D;

#ifdef PPX_ENABLE_BLOCK_ALLOCATOR
template <typename T> using allocator = kit::block_allocator<T>;
#else
template <typename T> using allocator = kit::vanilla_allocator<T>;
#endif
using quad_tree = kit::quad_tree<collider2D *, allocator>;
} // namespace ppx