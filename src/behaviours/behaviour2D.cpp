#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
behaviour2D::behaviour2D(const std::string &name, const std::size_t allocations) : kit::identifiable<std::string>(name)
{
    m_bodies.reserve(allocations);
}

void behaviour2D::validate()
{
    for (auto it = m_bodies.begin(); it != m_bodies.end();)
        if (!(*it))
            it = m_bodies.erase(it);
        else
            ++it;
}

void behaviour2D::add(const body2D::ptr &body)
{
    m_bodies.push_back(body);
}
void behaviour2D::remove(const body2D &body)
{
    for (auto it = m_bodies.begin(); it != m_bodies.end(); ++it)
        if (*(*it) == body)
        {
            m_bodies.erase(it);
            break;
        }
}
bool behaviour2D::contains(const body2D &body) const
{
    for (const body2D::ptr &bdptr : m_bodies)
        if (*bdptr == body)
            return true;
    return false;
}
float behaviour2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const auto &body : m_bodies)
        ke += body->kinetic_energy();
    return ke;
}

body2D::const_ptr behaviour2D::operator[](std::size_t index) const
{
    return m_bodies[index];
}
const body2D::ptr &behaviour2D::operator[](std::size_t index)
{
    return m_bodies[index];
}

float behaviour2D::energy(const body2D &body) const
{
    return body.kinetic_energy() + potential_energy(body);
}
float behaviour2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

void behaviour2D::clear()
{
    m_bodies.clear();
}
std::size_t behaviour2D::size() const
{
    return m_bodies.size();
}

void behaviour2D::apply_force_to_bodies()
{
    for (const auto &body : m_bodies)
    {
        const glm::vec3 f = force(*body);
        body->apply_simulation_force(glm::vec2(f));
        body->apply_simulation_torque(f.z);
    }
}

const std::vector<body2D::ptr> &behaviour2D::bodies() const
{
    return m_bodies;
}

#ifdef KIT_USE_YAML_CPP
YAML::Node behaviour2D::encode() const
{
    YAML::Node node;
    node["Enabled"] = enabled;

    for (const auto &body : m_bodies)
        node["Bodies"].push_back(body->index);
    node["Bodies"].SetStyle(YAML::EmitterStyle::Flow);
    return node;
}
bool behaviour2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() < 2 || !m_world)
        return false;
    clear();

    enabled = node["Enabled"].as<bool>();
    for (const YAML::Node &n : node["Bodies"])
        add(m_world->bodies.ptr(n.as<std::size_t>()));
    return true;
}
#endif
} // namespace ppx