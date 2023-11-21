#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/interaction2D.hpp"

namespace ppx
{
float interaction2D::potential(const body2D &body, const glm::vec2 &position) const
{
    m_unit.position(position);
    return potential_energy_pair(m_unit, body);
}
float interaction2D::potential(const glm::vec2 &position) const
{
    m_unit.position(position);
    float pot = 0.f;
    for (const auto &body : m_bodies)
        pot += potential_energy_pair(m_unit, *body);
    return pot;
}

float interaction2D::potential_energy(const body2D &body) const
{
    float pot = 0.f;
    for (const auto &bdptr : m_bodies)
        if (*bdptr != body)
            pot += potential_energy_pair(body, *bdptr);
    return pot;
}
float interaction2D::potential_energy() const
{
    float pot = 0.f;
    for (std::size_t i = 0; i < m_bodies.size(); i++)
        for (std::size_t j = i + 1; j < m_bodies.size(); j++)
            pot += potential_energy_pair(*m_bodies[i], *m_bodies[j]);
    return pot;
}

glm::vec3 interaction2D::force(const body2D &body1) const
{
    glm::vec3 total_force{0.f};
    for (const auto &body2 : m_bodies)
        if (body1 != *body2)
            total_force += force_pair(body1, *body2);

    return total_force;
}
} // namespace ppx