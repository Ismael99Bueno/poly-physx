#pragma once

#include "ppx/entities/shapes2D.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/vector_ptr.hpp"

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