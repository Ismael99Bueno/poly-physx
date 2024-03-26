#include "ppx/internal/pch.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
float pvconstraint2D::compute_velocity_lambda() const
{
    const float cvel = constraint_velocity();
    if (world.constraints.baumgarte_correction && std::abs(m_c) > world.constraints.baumgarte_threshold)
        return -(cvel + world.constraints.baumgarte_coef * m_c / world.integrator.ts.value) / m_inv_mass;

    return -cvel / m_inv_mass;
}

void pvconstraint2D::startup()
{
    vconstraint2D::startup();
    m_c = constraint_position();
}
} // namespace ppx