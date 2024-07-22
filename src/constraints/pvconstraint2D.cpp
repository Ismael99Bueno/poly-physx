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
    if (m_baumgarte && glm::length(m_c) > m_bthreshold)
        cvel += m_bcoeff * m_c / this->m_ts;

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
        const float signed_slop = m_c > 0.f ? -m_slop : m_slop;
        return -std::clamp(m_position_resolution_speed * (m_c + signed_slop), -m_max_position_correction,
                           m_max_position_correction) *
               this->m_mass;
    }
    else
    {
        const flat_t correction = m_position_resolution_speed * (glm::normalize(m_c) * m_slop - m_c);
        if (glm::length2(correction) > m_max_position_correction * m_max_position_correction)
            return this->m_mass * m_max_position_correction * glm::normalize(correction);

        return this->m_mass * correction;
    }
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
    else if constexpr (LinDegrees == 1)
        return this->m_dir * ccorrection;
    else
        return glm::vec2(ccorrection);
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

    const float inv_ts = 1.f / this->m_ts;
    if (this->m_dyn1)
    {
        state2D &st1 = this->state1();
        const glm::vec2 dpos1 = this->m_imass1 * lincorrection;
        const float da1 = this->m_iinertia1 * kit::cross2D(this->m_offset1, lincorrection);

        st1.centroid.position -= dpos1;
        st1.centroid.rotation -= da1;
        st1.velocity -= dpos1 * inv_ts;
        st1.angular_velocity -= da1 * inv_ts;
    }
    if (this->m_dyn2)
    {
        state2D &st2 = this->state2();
        const glm::vec2 dpos2 = this->m_imass2 * lincorrection;
        const float da2 = this->m_iinertia2 * kit::cross2D(this->m_offset2, lincorrection);

        st2.centroid.position += dpos2;
        st2.centroid.rotation += da2;
        st2.velocity += dpos2 * inv_ts;
        st2.angular_velocity += da2 * inv_ts;
    }
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void pvconstraint2D<LinDegrees, AngDegrees>::apply_angular_correction(float angcorrection)
{
    if constexpr (AngDegrees == 0)
    {
        KIT_ERROR("Angular correction can only be applied when the angular degrees of the constraint is equal to 1")
    }

    if (this->m_dyn1)
    {
        state2D &st1 = this->state1();
        const float da1 = this->m_iinertia1 * angcorrection;
        st1.centroid.rotation -= da1;
    }
    if (this->m_dyn2)
    {
        state2D &st2 = this->state2();
        const float da2 = this->m_iinertia2 * angcorrection;
        st2.centroid.rotation += da2;
    }
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void pvconstraint2D<LinDegrees, AngDegrees>::startup(std::vector<state2D> &states)
{
    vconstraint2D<LinDegrees, AngDegrees>::startup(states);

    // cache friendly stuff
    m_slop = this->world.joints.constraints.params.slop;
    m_baumgarte = this->world.joints.constraints.params.baumgarte_correction;
    m_bthreshold = this->world.joints.constraints.params.baumgarte_threshold;
    m_bcoeff = this->world.joints.constraints.params.baumgarte_coef;
    m_max_position_correction = this->world.joints.constraints.params.max_position_correction;
    m_position_resolution_speed = this->world.joints.constraints.params.position_resolution_speed;
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
bool pvconstraint2D<LinDegrees, AngDegrees>::solve_positions()
{
    update_position_data();
    if (glm::length(m_c) < m_slop)
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