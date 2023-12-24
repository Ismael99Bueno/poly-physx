#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision2D.hpp"

namespace ppx
{
const glm::vec2 &collision2D::touch1(std::size_t manifold_index) const
{
    KIT_ASSERT_ERROR(manifold_index < manifold.size, "Manifold index exceeds manifod size")
    return manifold.contacts[manifold_index];
}
glm::vec2 collision2D::touch2(std::size_t manifold_index) const
{
    KIT_ASSERT_ERROR(manifold_index < manifold.size, "Manifold index exceeds manifod size")
    return manifold.contacts[manifold_index] - mtv;
}
} // namespace ppx