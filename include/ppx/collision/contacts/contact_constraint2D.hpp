#pragma once

#include "ppx/constraints/pvconstraint2D.hpp"
#include "ppx/collision/contacts/contact2D.hpp"

namespace ppx
{
class contact_constraint2D : public contact2D, public pvconstraint2D<1, 0>
{
  protected:
    contact_constraint2D(world2D &world, const collision2D *collision, std::size_t manifold_index);
};
} // namespace ppx