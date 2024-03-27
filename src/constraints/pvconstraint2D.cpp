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

bool pvconstraint2D::adjust_clamped(const float min, const float max)
{
    return apply_corrections(std::clamp(constraint_position(), min, max));
}
bool pvconstraint2D::adjust_unclamped()
{
    return apply_corrections(constraint_position());
}

bool pvconstraint2D::apply_corrections(const float c)
{
    if (std::abs(c) < world.constraints.slop)
        return true;

    const float lambda = -c / m_inv_mass;

    const glm::vec2 imp2 = lambda * m_dir;
    const glm::vec2 imp1 = -imp2;

    m_body1->ctr_state.centroid.position += m_body1->props().dynamic.inv_mass * imp1;
    m_body2->ctr_state.centroid.position += m_body2->props().dynamic.inv_mass * imp2;

    m_body1->ctr_state.centroid.rotation += m_body1->props().dynamic.inv_inertia * kit::cross2D(m_offset1, imp1);
    m_body2->ctr_state.centroid.rotation += m_body2->props().dynamic.inv_inertia * kit::cross2D(m_offset2, imp2);

    m_body1->ctr_state.gposition = m_body1->ctr_state.global_centroid_point(m_body1->ctr_state.lposition);
    m_body2->ctr_state.gposition = m_body2->ctr_state.global_centroid_point(m_body2->ctr_state.lposition);

    const glm::vec2 f1 = imp1 / world.integrator.ts.value;
    const glm::vec2 f2 = imp2 / world.integrator.ts.value;
    const float torque1 = kit::cross2D(m_offset1, f1);
    const float torque2 = kit::cross2D(m_offset2, f2);

    m_body1->apply_simulation_force(f1);
    m_body1->apply_simulation_torque(torque1);
    m_body2->apply_simulation_force(f2);
    m_body2->apply_simulation_torque(torque2);

    return false;
}

void pvconstraint2D::startup()
{
    vconstraint2D::startup();
    m_c = constraint_position();
}
} // namespace ppx