#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/serialization/serialization.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{

constraint2D::constraint2D(world2D &world, const char *name)
    : kit::identifiable<>(kit::uuid::random()), nameable(name), worldref2D(world)
{
}

bool constraint2D::contains(const body2D &body) const
{
    return contains(body.id);
}

#ifdef KIT_USE_YAML_CPP
YAML::Node constraint2D::encode() const
{
    return kit::yaml::codec<constraint2D>::encode(*this);
}
bool constraint2D::decode(const YAML::Node &node)
{
    return kit::yaml::codec<constraint2D>::decode(node, *this);
}
#endif
} // namespace ppx