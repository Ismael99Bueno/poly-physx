#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint2D.hpp"
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

void constraint2D::aggregate_impulse(body2D &body, const glm::vec2 &impulse)
{
    if (!body.kinematic)
        return;
    body.boost(body.inverse_mass() * impulse);
}
void constraint2D::aggregate_impulse(body2D &body, const glm::vec2 &impulse, const glm::vec2 &anchor)
{
    if (!body.kinematic)
        return;
    body.boost(body.inverse_mass() * impulse);
    body.spin(body.inverse_inertia() * kit::cross2D(anchor, impulse));
}

void constraint2D::apply_impulse(const body2D &body, const glm::vec2 &impulse, std::vector<float> &state_derivative)
{
    if (!body.kinematic)
        return;
    const glm::vec2 dv = body.inverse_mass() * impulse;
    state_derivative[6 * body.index] += dv.x;
    state_derivative[6 * body.index + 1] += dv.y;
}
void constraint2D::apply_impulse(const body2D &body, const glm::vec2 &impulse, const glm::vec2 &anchor,
                                 std::vector<float> &state_derivative)
{
    if (!body.kinematic)
        return;
    apply_impulse(body, impulse, state_derivative);
    state_derivative[6 * body.index + 2] += body.inverse_inertia() * kit::cross2D(anchor, impulse);
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