#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
bool collision2D::add_contact_point(const glm::vec2 &contact)
{
    KIT_ASSERT_ERROR(size < MANIFOLD_SIZE, "Cannot add more contact points to collision")
    for (std::size_t i = 0; i < size; i++)
        if (glm::distance2(contact, manifold[i]) <= MIN_CONTACT_DIST * MIN_CONTACT_DIST)
            return false;

    manifold[size++] = contact;
    return true;
}

const glm::vec2 &collision2D::touch1(std::size_t manifold_index) const
{
    return manifold[manifold_index];
}
glm::vec2 collision2D::touch2(std::size_t manifold_index) const
{
    return manifold[manifold_index] - normal;
}
} // namespace ppx