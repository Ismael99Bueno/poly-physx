#include "ppx/internal/pch.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
float pvconstraint2D::compute_velocity_lambda() const
{
    const float cvel = constraint_velocity();
    if (world.constraints.baumgarte_correction && std::abs(m_c) > world.constraints.baumgarte_threshold)
        return -(cvel + world.constraints.baumgarte_coef * m_c / world.rk_substep_timestep()) / m_inv_mass;

    return -cvel / m_inv_mass;
}

bool pvconstraint2D::adjust_positions()
{
    startup();
    if (std::abs(m_c) < world.constraints.slop)
        return true;

    const float signed_slop = m_c > 0.f ? -world.constraints.slop : world.constraints.slop;
    const float lambda =
        -std::clamp(world.constraints.overlap_resolution_speed * (m_c + signed_slop),
                    -world.constraints.max_position_correction, world.constraints.max_position_correction) /
        m_inv_mass;

    const glm::vec2 imp2 = lambda * m_dir;
    const glm::vec2 imp1 = -imp2;

    const glm::vec2 dpos1 = m_body1->props().dynamic.inv_mass * imp1;
    const glm::vec2 dpos2 = m_body2->props().dynamic.inv_mass * imp2;

    const float da1 = m_body1->props().dynamic.inv_inertia * kit::cross2D(m_offset1, imp1);
    const float da2 = m_body2->props().dynamic.inv_inertia * kit::cross2D(m_offset2, imp2);

    m_body1->ctr_state.centroid.translate(dpos1);
    m_body2->ctr_state.centroid.translate(dpos2);

    m_body1->ctr_state.centroid.rotate(da1);
    m_body2->ctr_state.centroid.rotate(da2);

    m_body1->ctr_state.gposition = m_body1->ctr_state.global_centroid_point(m_body1->ctr_state.lposition);
    m_body2->ctr_state.gposition = m_body2->ctr_state.global_centroid_point(m_body2->ctr_state.lposition);

    m_body1->velocity() += dpos1 / world.rk_substep_timestep();
    m_body1->angular_velocity() += da1 / world.rk_substep_timestep();
    m_body2->velocity() += dpos2 / world.rk_substep_timestep();
    m_body2->angular_velocity() += da2 / world.rk_substep_timestep();
    return false;
}

void pvconstraint2D::startup()
{
    vconstraint2D::startup();
    m_c = constraint_position();
}
} // namespace ppx