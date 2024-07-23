#pragma once

#include "geo/shapes/polytope.hpp"
#include "geo/shapes/nsphere.hpp"
#include "geo/algorithm/ray.hpp"
#ifdef PPX_ENABLE_BLOCK_ALLOCATOR
#include "kit/memory/allocator/block_allocator.hpp"
#else
#include "kit/memory/allocator/vanilla_allocator.hpp"
#endif
#include "kit/container/quad_tree.hpp"

#ifndef PPX_MAX_VERTICES
#define PPX_MAX_VERTICES 8
#endif

#ifndef PPX_MAX_QUAD_COLLIDERS
#define PPX_MAX_QUAD_COLLIDERS 128
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
template <typename T> using allocator_t = kit::block_allocator<T>;
#else
template <typename T> using allocator_t = kit::vanilla_allocator<T>;
#endif

template <typename T> class allocator
{
  public:
    template <class... Args> static T *create(Args &&...args)
    {
        return s_allocator.create(std::forward<Args>(args)...);
    }
    static void destroy(T *ptr)
    {
        s_allocator.destroy(ptr);
    }

  private:
    static inline allocator_t<T> s_allocator;
};

using quad_tree = kit::quad_tree<collider2D *, PPX_MAX_QUAD_COLLIDERS, allocator_t>;

} // namespace ppx