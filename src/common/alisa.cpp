#include "ppx/internal/pch.hpp"
#include "ppx/common/alias.hpp"
#include "ppx/collider/collider2D.hpp"

namespace ppx
{
const geo::aabb2D &qt_element::operator()() const
{
    return collider->bounding_box();
}
} // namespace ppx