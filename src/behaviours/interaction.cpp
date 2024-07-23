#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/interaction.hpp"
#include "ppx/world.hpp"

namespace ppx
{
interaction2D::interaction2D(world2D &world, const std::string &name) : behaviour2D(world, name)
{
}
float interaction2D::potential(const state2D &state, const glm::vec2 &position) const
{
    m_unit.centroid.position = position;
    return potential_energy_pair(m_unit, state);
}
float interaction2D::potential(const glm::vec2 &position) const
{
    m_unit.centroid.position = position;
    float pot = 0.f;
    for (const state2D &state : world.bodies.states())
        pot += potential_energy_pair(m_unit, state);
    return pot;
}

float interaction2D::potential_energy(const state2D &state1) const
{
    float pot = 0.f;
    for (const state2D &state2 : world.bodies.states())
        if (&state1 != &state2)
            pot += potential_energy_pair(state1, state2);
    return pot;
}
float interaction2D::potential_energy() const
{
    float pot = 0.f;
    for (std::size_t i = 0; i < m_elements.size(); i++)
        for (std::size_t j = i + 1; j < m_elements.size(); j++)
            pot += potential_energy_pair(m_elements[i]->state(), m_elements[j]->state());
    return pot;
}

bool interaction2D::add(body2D *body)
{
    if (!behaviour2D::add(body))
        return false;
    if (m_enabled)
        for (body2D *b : m_elements)
            b->awake();
    return true;
}

bool interaction2D::remove(std::size_t index)
{
    if (!behaviour2D::remove(index))
        return false;
    if (m_enabled)
        for (body2D *body : m_elements)
            body->awake();
    return true;
}

glm::vec3 interaction2D::force(const state2D &state1) const
{
    glm::vec3 total_force{0.f};
    for (const state2D &state2 : world.bodies.states())
        if (&state1 != &state2)
            total_force += force_pair(state1, state2);

    return total_force;
}
} // namespace ppx