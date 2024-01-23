#pragma once

#include "geo/shapes2D/polygon.hpp"
#include "geo/shapes2D/circle.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/vector_ptr.hpp"

#ifndef PPX_MAX_VERTICES
#define PPX_MAX_VERTICES 8
#endif

namespace ppx
{
class body2D;

class collider2D : public kit::identifiable<>, public kit::indexable
{
  public:
    using ptr = kit::vector_ptr<collider2D>;
    using const_ptr = kit::const_vector_ptr<collider2D>;
};
} // namespace ppx