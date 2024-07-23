#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint.hpp"

namespace ppx
{
constraint2D::constraint2D(const specs::constraint2D::properties &cprops)
    : m_is_soft(cprops.is_soft), m_frequency(cprops.frequency), m_damping_ratio(cprops.damping_ratio)
{
}

specs::constraint2D::properties constraint2D::cprops() const
{
    specs::constraint2D::properties cprops;
    fill_cprops(cprops);
    return cprops;
}
void constraint2D::cprops(const specs::constraint2D::properties &cprops)
{
    jprops(cprops);
    m_is_soft = cprops.is_soft;
    m_frequency = cprops.frequency;
    m_damping_ratio = cprops.damping_ratio;
}

void constraint2D::fill_cprops(specs::constraint2D::properties &cprops) const
{
    fill_jprops(cprops);
    cprops.is_soft = m_is_soft;
    cprops.frequency = m_frequency;
    cprops.damping_ratio = m_damping_ratio;
}

bool constraint2D::is_soft() const
{
    return m_is_soft;
}
void constraint2D::is_soft(const bool is_soft)
{
    m_is_soft = is_soft;
    awake();
}

float constraint2D::frequency() const
{
    return m_frequency;
}
void constraint2D::frequency(float frequency)
{
    m_frequency = frequency;
    awake();
}

float constraint2D::damping_ratio() const
{
    return m_damping_ratio;
}
void constraint2D::damping_ratio(float damping_ratio)
{
    m_damping_ratio = damping_ratio;
    awake();
}

bool constraint2D::solve_positions()
{
    return true;
}

bool constraint2D::is_constraint() const
{
    return true;
}

} // namespace ppx