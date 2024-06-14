#include "ppx/internal/pch.hpp"
#include "ppx/collision/manifold/pp_manifold_algorithm2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
pp_manifold_algorithm2D::pp_manifold_algorithm2D(world2D &world) : worldref2D(world)
{
}
} // namespace ppx