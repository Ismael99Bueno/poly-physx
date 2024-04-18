#include "ppx/internal/pch.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
typename pvconstraint2D<LinDegrees, AngDegrees>::flat_t pvconstraint2D<LinDegrees,
                                                                       AngDegrees>::compute_constraint_impulse() const
{
    flat_t cvel = this->constraint_velocity();
    if (this->world.constraints.baumgarte_correction && glm::length(m_c) > this->world.constraints.baumgarte_threshold)
        cvel += this->world.constraints.baumgarte_coef * m_c / this->world.rk_substep_timestep();

    if constexpr (LinDegrees + AngDegrees == 1)
        return -cvel * this->m_mass;
    else
        return this->m_mass * (-cvel);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
typename pvconstraint2D<LinDegrees, AngDegrees>::flat_t pvconstraint2D<
    LinDegrees, AngDegrees>::compute_constraint_correction() const
{
    if constexpr (LinDegrees + AngDegrees == 1)
    {
        const float signed_slop = m_c > 0.f ? -this->world.constraints.slop : this->world.constraints.slop;
        return -std::clamp(this->world.constraints.position_resolution_speed * (m_c + signed_slop),
                           -this->world.constraints.max_position_correction,
                           this->world.constraints.max_position_correction) *
               this->m_mass;
    }
    else
        return this->m_mass * (glm::normalize(m_c) * this->world.constraints.slop - m_c);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
glm::vec2 pvconstraint2D<LinDegrees, AngDegrees>::compute_linear_correction(const flat_t &ccorrection) const
{
    if constexpr (LinDegrees == 0)
    {
        KIT_ERROR("Linear correction can only be computed when the linear degrees of the constraint is greater than 0")
        return {};
    }
    else if constexpr (LinDegrees + AngDegrees > 1)
        return glm::vec2(ccorrection);
    else
        return this->m_dir * ccorrection;
}
template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
float pvconstraint2D<LinDegrees, AngDegrees>::compute_angular_correction(const flat_t &ccorrection) const
{
    if constexpr (AngDegrees == 0)
    {
        KIT_ERROR("Angular correction can only be computed when the angular degrees of the constraint is equal to 1")
        return 0.f;
    }
    else if constexpr (LinDegrees == 2)
        return ccorrection.z;
    else if constexpr (LinDegrees == 1)
        return ccorrection.y;
    else
        return ccorrection;
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void pvconstraint2D<LinDegrees, AngDegrees>::apply_linear_correction(const glm::vec2 &lincorrection)
{
    if constexpr (LinDegrees == 0)
    {
        KIT_ERROR("Linear correction can only be applied when the linear degrees of the constraint is greater than 0")
    }
    const glm::vec2 dpos1 = -this->m_body1->props().dynamic.inv_mass * lincorrection;
    const glm::vec2 dpos2 = this->m_body2->props().dynamic.inv_mass * lincorrection;

    const float da1 = -this->m_body1->props().dynamic.inv_inertia * kit::cross2D(this->m_offset1, lincorrection);
    const float da2 = this->m_body2->props().dynamic.inv_inertia * kit::cross2D(this->m_offset2, lincorrection);

    this->m_body1->ctr_state.centroid.translate(dpos1);
    this->m_body2->ctr_state.centroid.translate(dpos2);

    this->m_body1->ctr_state.centroid.rotate(da1);
    this->m_body2->ctr_state.centroid.rotate(da2);

    this->m_body1->velocity() += dpos1 / this->world.rk_substep_timestep();
    this->m_body1->angular_velocity() += da1 / this->world.rk_substep_timestep();
    this->m_body2->velocity() += dpos2 / this->world.rk_substep_timestep();
    this->m_body2->angular_velocity() += da2 / this->world.rk_substep_timestep();
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void pvconstraint2D<LinDegrees, AngDegrees>::apply_angular_correction(float angcorrection)
{
    if constexpr (AngDegrees == 0)
    {
        KIT_ERROR("Angular correction can only be applied when the angular degrees of the constraint is equal to 1")
    }
    const float da1 = -angcorrection;
    const float da2 = angcorrection;

    this->m_body1->ctr_state.centroid.rotate(da1);
    this->m_body2->ctr_state.centroid.rotate(da2);

    this->m_body1->angular_velocity() += da1 / this->world.rk_substep_timestep();
    this->m_body2->angular_velocity() += da2 / this->world.rk_substep_timestep();
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
bool pvconstraint2D<LinDegrees, AngDegrees>::solve_positions()
{
    update_position_data();
    if (glm::length(m_c) < this->world.constraints.slop)
        return true;

    const flat_t ccorrection = compute_constraint_correction();
    if constexpr (LinDegrees > 0)
        apply_linear_correction(compute_linear_correction(ccorrection));
    if constexpr (AngDegrees == 1)
        apply_angular_correction(compute_angular_correction(ccorrection));

    return false;
}
template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void pvconstraint2D<LinDegrees, AngDegrees>::solve()
{
    this->solve_velocities();
    solve_positions();
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void pvconstraint2D<LinDegrees, AngDegrees>::update_constraint_data()
{
    vconstraint2D<LinDegrees, AngDegrees>::update_constraint_data();
    m_c = constraint_position();
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void pvconstraint2D<LinDegrees, AngDegrees>::update_position_data()
{
    update_constraint_data();
}

template class pvconstraint2D<1, 0>;
template class pvconstraint2D<0, 1>;
template class pvconstraint2D<1, 1>;
template class pvconstraint2D<2, 0>;
template class pvconstraint2D<2, 1>;

} // namespace ppx