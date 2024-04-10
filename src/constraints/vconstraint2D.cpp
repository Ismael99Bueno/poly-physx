#include "ppx/internal/pch.hpp"
#include "ppx/constraints/vconstraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
typename vconstraint2D<LinDegrees, AngDegrees>::flat_t vconstraint2D<LinDegrees,
                                                                     AngDegrees>::compute_constraint_impulse() const
{
    if constexpr (LinDegrees + AngDegrees == 1)
        return -constraint_velocity() * m_mass;
    else
        return m_mass * (-constraint_velocity());
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
glm::vec2 vconstraint2D<LinDegrees, AngDegrees>::compute_linear_impulse(const flat_t &cimpulse) const
{
    if constexpr (LinDegrees + AngDegrees > 1)
        return glm::vec2(cimpulse);
    else
    {
        KIT_ERROR("compute_linear_impulse must be specialized when the total degrees of the constraint is 1")
        return glm::vec2(0.f);
    }
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
float vconstraint2D<LinDegrees, AngDegrees>::compute_angular_impulse(const flat_t &cimpulse) const
{
    if constexpr (AngDegrees == 0)
    {
        KIT_ERROR("Angular impulse can only be computed when the angular degrees of the constraint is equal to 1")
        return 0.f;
    }
    else if constexpr (LinDegrees == 2)
        return cimpulse.z;
    else if constexpr (LinDegrees == 1)
        return cimpulse.y;
    else
        return cimpulse;
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::apply_linear_impulse(const glm::vec2 &linimpulse)
{
    if constexpr (LinDegrees == 0)
    {
        KIT_ERROR("Linear impulse can only be applied when the linear degrees of the constraint is greater than 0")
    }
    m_body1->ctr_state.velocity -= m_body1->props().dynamic.inv_mass * linimpulse;
    m_body2->ctr_state.velocity += m_body2->props().dynamic.inv_mass * linimpulse;

    m_body1->ctr_state.angular_velocity -= m_body1->props().dynamic.inv_inertia * kit::cross2D(m_offset1, linimpulse);
    m_body2->ctr_state.angular_velocity += m_body2->props().dynamic.inv_inertia * kit::cross2D(m_offset2, linimpulse);

    const glm::vec2 f1 = -linimpulse / world.rk_substep_timestep();
    const glm::vec2 f2 = linimpulse / world.rk_substep_timestep();
    const float torque1 = kit::cross2D(m_offset1, f1);
    const float torque2 = kit::cross2D(m_offset2, f2);

    m_body1->apply_simulation_force(f1);
    m_body1->apply_simulation_torque(torque1);
    m_body2->apply_simulation_force(f2);
    m_body2->apply_simulation_torque(torque2);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::apply_angular_impulse(float angimpulse)
{
    if constexpr (AngDegrees == 0)
    {
        KIT_ERROR("Angular impulse can only be applied when the angular degrees of the constraint is equal to 1")
    }
    m_body1->ctr_state.angular_velocity -= m_body1->props().dynamic.inv_inertia * angimpulse;
    m_body2->ctr_state.angular_velocity += m_body2->props().dynamic.inv_inertia * angimpulse;

    const float torque1 = -angimpulse / world.rk_substep_timestep();
    const float torque2 = angimpulse / world.rk_substep_timestep();

    m_body1->apply_simulation_torque(torque1);
    m_body2->apply_simulation_torque(torque2);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::solve()
{
    solve_velocities();
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::solve_velocities()
{
    const flat_t impulse = compute_constraint_impulse();
    m_cumimpulse += impulse;
    if constexpr (LinDegrees > 0)
        apply_linear_impulse(compute_linear_impulse(impulse));
    if constexpr (AngDegrees == 1)
        apply_angular_impulse(compute_angular_impulse(impulse));
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::solve_velocities_clamped(const flat_t &min, const flat_t &max)
{
    const flat_t impulse = compute_constraint_impulse();
    const flat_t old_impulse = m_cumimpulse;
    m_cumimpulse = glm::clamp(m_cumimpulse + impulse, min, max);

    const flat_t delta_impulse = m_cumimpulse - old_impulse;
    if constexpr (LinDegrees > 0)
    {
        const glm::vec2 linimpulse = compute_linear_impulse(delta_impulse);
        if (!kit::approaches_zero(glm::length2(linimpulse)))
            apply_linear_impulse(compute_linear_impulse(delta_impulse));
    }
    if constexpr (AngDegrees == 1)
    {
        const float angimpulse = compute_angular_impulse(delta_impulse);
        if (!kit::approaches_zero(angimpulse))
            apply_angular_impulse(compute_angular_impulse(delta_impulse));
    }
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::startup()
{
    update_constraint_data();
    if (world.constraints.warmup)
        warmup();
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::update_constraint_data()
{
    m_ganchor1 = m_body1->ctr_state.global_position_point(m_lanchor1);
    if (!m_single_anchor)
        m_ganchor2 = m_body2->ctr_state.global_position_point(m_lanchor2);
    else
        m_ganchor2 = m_ganchor1;

    m_offset1 = m_ganchor1 - m_body1->ctr_state.centroid.position();
    m_offset2 = m_ganchor2 - m_body2->ctr_state.centroid.position();
    if constexpr (LinDegrees + AngDegrees == 1)
        m_mass = 1.f / inverse_mass();
    else
        m_mass = glm::inverse(inverse_mass());
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::warmup()
{
    if (kit::approaches_zero(glm::length(m_cumimpulse)))
        return;
    if constexpr (LinDegrees > 0)
        apply_linear_impulse(compute_linear_impulse(m_cumimpulse));
    if constexpr (AngDegrees == 1)
        apply_angular_impulse(compute_angular_impulse(m_cumimpulse));
}

float vconstraint10_2D::inverse_mass() const
{
    const float cross1 = kit::cross2D(m_offset1, m_dir);
    const float cross2 = kit::cross2D(m_offset2, m_dir);
    return m_body1->props().dynamic.inv_mass + m_body2->props().dynamic.inv_mass +
           m_body1->props().dynamic.inv_inertia * cross1 * cross1 +
           m_body2->props().dynamic.inv_inertia * cross2 * cross2;
}

void vconstraint10_2D::update_constraint_data()
{
    m_ganchor1 = m_body1->ctr_state.global_position_point(m_lanchor1);
    if (!m_single_anchor)
        m_ganchor2 = m_body2->ctr_state.global_position_point(m_lanchor2);
    else
        m_ganchor2 = m_ganchor1;

    m_offset1 = m_ganchor1 - m_body1->ctr_state.centroid.position();
    m_offset2 = m_ganchor2 - m_body2->ctr_state.centroid.position();
    m_dir = direction();
    m_mass = 1.f / inverse_mass();
}

glm::vec2 vconstraint10_2D::compute_linear_impulse(const float &cimpulse) const
{
    return cimpulse * m_dir;
}

float vconstraint01_2D::inverse_mass() const
{
    return m_body1->props().dynamic.inv_inertia + m_body2->props().dynamic.inv_inertia;
}

template class vconstraint2D<1, 0>;
template class vconstraint2D<0, 1>;
template class vconstraint2D<1, 1>;
template class vconstraint2D<2, 0>;
template class vconstraint2D<2, 1>;

} // namespace ppx