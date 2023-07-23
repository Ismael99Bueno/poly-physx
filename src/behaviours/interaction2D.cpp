#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/interaction2D.hpp"

namespace ppx
{
float interaction2D::potential(const body2D &bd, const glm::vec2 &position) const
{
    m_unit.position(position);
    return potential_energy_pair(m_unit, bd);
}
float interaction2D::potential(const glm::vec2 &position) const
{
    m_unit.position(position);
    float pot = 0.f;
    for (const auto &bd : m_included)
        pot += potential_energy_pair(m_unit, *bd);
    return pot;
}

float interaction2D::potential_energy(const body2D &bd) const
{
    float pot = 0.f;
    for (const auto &bdptr : m_included)
        if (*bdptr != bd)
            pot += potential_energy_pair(bd, *bdptr);
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

std::pair<glm::vec2, float> interaction2D::force(const body2D &bd1) const
{
    glm::vec2 total_force{0.f};
    float total_torque = 0.f;
    for (const auto &bd2 : m_included)
        if (bd1 != *bd2)
        {
            const auto &[f, t] = force_pair(bd1, *bd2);
            total_force += f;
            total_torque += t;
        }
    return {total_force, total_torque};
}
} // namespace ppx