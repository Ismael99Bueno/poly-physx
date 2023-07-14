#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/interaction2D.hpp"

namespace ppx
{
float interaction2D::potential(const entity2D &e, const glm::vec2 &pos) const
{
    m_unit.pos(pos);
    return potential_energy_pair(m_unit, e);
}
float interaction2D::potential(const glm::vec2 &pos) const
{
    m_unit.pos(pos);
    float pot = 0.f;
    for (const auto &e : m_included)
        pot += potential_energy_pair(m_unit, *e);
    return pot;
}

float interaction2D::potential_energy(const entity2D &e) const
{
    float pot = 0.f;
    for (const auto &entt : m_included)
        if (*entt != e)
            pot += potential_energy_pair(e, *entt);
    return pot;
}
float interaction2D::potential_energy() const
{
    float pot = 0.f;
    for (std::size_t i = 0; i < m_included.size(); i++)
        for (std::size_t j = i + 1; j < m_included.size(); j++)
            pot += potential_energy_pair(*m_included[i], *m_included[j]);
    return pot;
}

std::pair<glm::vec2, float> interaction2D::force(const entity2D &e1) const
{
    glm::vec2 total_force{0.f};
    float total_torque = 0.f;
    for (const auto &e2 : m_included)
        if (e1 != *e2)
        {
            const auto &[f, t] = force_pair(e1, *e2);
            total_force += f;
            total_torque += t;
        }
    return {total_force, total_torque};
}
} // namespace ppx