#include "force2D.hpp"

namespace ppx
{
    float force2D::potential_energy() const
    {
        float pot = 0.f;
        for (const auto &e : m_entities)
            pot += potential_energy(*e);
        return pot;
    }
    float force2D::energy(const ppx::entity2D &e) const { return e.kinetic_energy() + potential_energy(e); }
    float force2D::energy() const { return kinetic_energy() + potential_energy(); }
}