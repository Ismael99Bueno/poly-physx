#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{

constraint2D::constraint2D(const char *name) : nameable(name)
{
}

bool constraint2D::contais(const body2D &body) const
{
    return contains(body.id);
}

#ifdef KIT_USE_YAML_CPP
YAML::Node constraint2D::encode() const
{
    YAML::Node node;
    node["UUID"] = (std::uint64_t)id;

    return node;
}
bool constraint2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() < 1)
        return false;
    id = kit::uuid(node["UUID"].as<std::uint64_t>());
    return true;
}
#endif
} // namespace ppx