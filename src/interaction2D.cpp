#include "interaction2D.hpp"

namespace phys
{
    float interaction2D::potential(const entity2D &e, const alg::vec2 &pos) const
    {
        m_unit.pos(pos);
        return potential_energy_pair(m_unit, e);
    }
    float interaction2D::potential(const alg::vec2 &pos) const
    {
        m_unit.pos(pos);
        float pot = 0.f;
        for (const auto &e : m_entities)
            pot += potential_energy_pair(m_unit, *e);
        return pot;
    }

    float interaction2D::potential_energy(const entity2D &e) const
    {
        float pot = 0.f;
        for (const auto &entt : m_entities)
            if (*entt != e)
                pot += potential_energy_pair(e, *entt);
        return pot;
    }
    float interaction2D::potential_energy() const
    {
        float pot = 0.f;
        for (std::size_t i = 0; i < m_entities.size(); i++)
            for (std::size_t j = i; j < m_entities.size(); j++)
                pot += potential_energy_pair(*m_entities[i], *m_entities[j]);
        return pot;
    }

    float interaction2D::energy(const entity2D &e) const { return e.kinetic_energy() + potential_energy(e); }
    float interaction2D::energy() const { return kinetic_energy() + potential_energy(); }
}