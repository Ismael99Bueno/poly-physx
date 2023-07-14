#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/force2D.hpp"

namespace ppx
{
float force2D::potential_energy() const
{
    float pot = 0.f;
    for (const auto &e : m_included)
        pot += potential_energy(*e);
    return pot;
}
} // namespace ppx