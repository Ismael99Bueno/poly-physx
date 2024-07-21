#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/world2D.hpp"
#ifdef KIT_USE_YAML_CPP
#include "ppx/serialization/serialization.hpp"
#endif

namespace ppx
{
behaviour2D::behaviour2D(world2D &world, const std::string &name)
    : manager2D(world), kit::identifiable<std::string>(name)
{
}

bool behaviour2D::add(body2D *body)
{
    if (contains(body))
        return false;
    m_elements.push_back(body);
    return true;
}
bool behaviour2D::remove(std::size_t index)
{
    if (index >= m_elements.size())
        return false;
    m_elements.erase(m_elements.begin() + index);
    return true;
}

float behaviour2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const auto &body : m_elements)
        ke += body->kinetic_energy();
    return ke;
}

float behaviour2D::energy(const body2D &body) const
{
    return body.kinetic_energy() + potential_energy(body.state());
}
float behaviour2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

void behaviour2D::load_forces(std::vector<state2D> &states) const
{
    for (const body2D *body : m_elements)
    {
        if (!body->is_dynamic() || body->asleep())
            continue;

        state2D &state = states[body->meta.index];
        const glm::vec3 f = force(state);

        state.substep_force += glm::vec2(f);
        state.substep_torque += f.z;
    }
}

#ifdef KIT_USE_YAML_CPP
YAML::Node behaviour2D::encode() const
{
    return kit::yaml::codec<behaviour2D>::encode(*this);
}
bool behaviour2D::decode(const YAML::Node &node)
{
    return kit::yaml::codec<behaviour2D>::decode(node, *this);
}
#endif
} // namespace ppx