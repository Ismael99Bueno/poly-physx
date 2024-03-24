#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/world2D.hpp"
#ifdef KIT_USE_YAML_CPP
#include "ppx/serialization/serialization.hpp"
#endif

namespace ppx
{
behaviour2D::behaviour2D(world2D &world, const std::string &name)
    : kit::identifiable<std::string>(name), worldref2D(world)
{
}

void behaviour2D::add(body2D *body)
{
    m_bodies.push_back(body);
}
bool behaviour2D::remove(std::size_t index)
{
    if (index >= m_bodies.size())
        return false;
    m_bodies.erase(m_bodies.begin() + index);
    return true;
}

bool behaviour2D::remove(const body2D *body)
{
    for (std::size_t i = 0; i < m_bodies.size(); i++)
        if (m_bodies[i] == body)
        {
            m_bodies.erase(m_bodies.begin() + i);
            return true;
        }
    return false;
}
bool behaviour2D::contains(const body2D *body) const
{
    for (body2D *b : m_bodies)
        if (b == body)
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

const body2D &behaviour2D::operator[](std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds container size: {0}", index)
    return *m_bodies[index];
}
body2D &behaviour2D::operator[](std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds container size: {0}", index)
    return *m_bodies[index];
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
        if (!body->is_dynamic())
            continue;
        const glm::vec3 f = force(*body);
        body->apply_simulation_force(glm::vec2(f));
        body->apply_simulation_torque(f.z);
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