#include "ppx/internal/pch.hpp"
#include "ppx/common/alias.hpp"
#include "ppx/collider/collider2D.hpp"

namespace ppx
{
const geo::aabb2D &qt_element::operator()() const
{
    return collider->bounding_box();
}
bool qt_element::operator==(const qt_element &other) const
{
    return collider == other.collider;
}
} // namespace ppx

namespace std
{
std::size_t hash<ppx::qt_element>::operator()(const ppx::qt_element &element) const
{
    return std::hash<ppx::collider2D *>()(element.collider);
}
} // namespace std
