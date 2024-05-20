#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/contact_constraint2D.hpp"

namespace ppx
{
contact_constraint2D::contact_constraint2D(world2D &world, const collision2D *collision, std::size_t manifold_index)
    : contact2D(collision, manifold_index),
      pvconstraint2D<1, 0>(world, m_collider1->body(), m_collider2->body(), collision->manifold[manifold_index].point)
{
}

} // namespace ppx