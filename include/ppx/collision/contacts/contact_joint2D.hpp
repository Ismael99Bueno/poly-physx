#pragma once

#include "ppx/joints/joint2D.hpp"
#include "ppx/collision/contacts/contact2D.hpp"

namespace ppx
{
class contact_joint2D : public contact2D, public joint2D
{
  protected:
    contact_joint2D(world2D &world, const collision2D *collision, std::size_t manifold_index);
};
} // namespace ppx