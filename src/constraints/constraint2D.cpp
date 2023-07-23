#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint2D.hpp"

namespace ppx
{
constraint2D::constraint2D(const float stiffness, const float dampening)
    : m_stiffness(stiffness), m_dampening(dampening)
{
}

float constraint2D::stiffness() const
{
    return m_stiffness;
}
float constraint2D::dampening() const
{
    return m_dampening;
}

void constraint2D::stiffness(float stiffness)
{
    m_stiffness = stiffness;
}
void constraint2D::dampening(float dampening)
{
    m_dampening = dampening;
}
#ifdef KIT_USE_YAML_CPP
YAML::Node constraint2D::encode() const
{
    YAML::Node node;
    node["UUID"] = (std::uint64_t)id();
    node["Stiffness"] = m_stiffness;
    node["Dampening"] = m_dampening;

    return node;
}
bool constraint2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() < 3)
        return false;
    id(node["UUID"].as<std::uint64_t>());
    m_stiffness = node["Stiffness"].as<float>();
    m_dampening = node["Dampening"].as<float>();

    return true;
}
#endif
} // namespace ppx