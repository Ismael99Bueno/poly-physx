#include "ppx/internal/pch.hpp"
#include "ppx/behaviours/force2D.hpp"

namespace ppx
{
float force2D::potential_energy() const
{
    float pot = 0.f;
    for (const auto &bd : m_included)
        pot += potential_energy(*bd);
    return pot;
}
} // namespace ppx