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

collision2D collision2D::reciprocal() const
{
    collision2D c = {incoming, current, -normal, {}, size};
    for (std::size_t i = 0; i < size; i++)
        c.manifold[i] = touch2(i);
    return c;
}

const glm::vec2 &collision2D::touch1(std::size_t manifold_index) const
{
    return manifold[manifold_index];
}
glm::vec2 collision2D::touch2(std::size_t manifold_index) const
{
    return manifold[manifold_index] - normal;
}

contact_point_query::contact_point_query(const glm::vec2 &first_contact)
{
    contacts[0] = first_contact;
}

void contact_point_query::add_contact_point(const glm::vec2 &contact)
{
    contacts[next] = contact;
    next = (next + 1) % (collision2D::MANIFOLD_SIZE - 1);
    if (size < collision2D::MANIFOLD_SIZE - 1)
        size++;
}
} // namespace ppx