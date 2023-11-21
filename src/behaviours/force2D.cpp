#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/force2D.hpp"

namespace ppx
{
float force2D::potential_energy() const
{
    float pot = 0.f;
    for (const auto &body : m_bodies)
        pot += potential_energy(*body);
    return pot;
}
} // namespace ppx