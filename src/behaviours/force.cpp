#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/force.hpp"

namespace ppx
{
float force2D::potential_energy() const
{
    float pot = 0.f;
    for (const body2D *body : m_elements)
        pot += potential_energy(body->state());
    return pot;
}
} // namespace ppx