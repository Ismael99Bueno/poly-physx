#include "ppx/internal/pch.hpp"
#include "ppx/constraints/vconstraint2D.hpp"
#include "ppx/joints/spring_joint2D.hpp"
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
    if constexpr (LinDegrees == 0)
    {
        KIT_ERROR("Linear impulse can only be computed when the linear degrees of the constraint is greater than 0")
        return glm::vec2(0.f);
    }
    else if constexpr (LinDegrees == 1 && AngDegrees == 0)
        return cimpulse * this->m_dir;
    else if constexpr (LinDegrees == 1 && AngDegrees == 1)
        return cimpulse.x * this->m_dir;
    else if constexpr (LinDegrees == 2)
        return glm::vec2(cimpulse);
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
    const auto apply = [this, &linimpulse](state2D &state, const float imass, const float iinertia,
                                           const glm::vec2 &offset) {
        state.velocity += imass * linimpulse;
        if (!m_no_anchors)
            state.angular_velocity += iinertia * kit::cross2D(offset, linimpulse);
    };

    m_force = linimpulse / m_ts;
    if (m_dyn1)
        apply(state1(), -m_imass1, -m_iinertia1, m_offset1);
    if (m_dyn2)
        apply(state2(), m_imass2, m_iinertia2, m_offset2);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::apply_angular_impulse(const float angimpulse)
{
    if constexpr (AngDegrees == 0)
    {
        KIT_ERROR("Angular impulse can only be applied when the angular degrees of the constraint is equal to 1")
    }
    m_torque = angimpulse / m_ts;
    if (m_dyn1)
    {
        state2D &st1 = state1();
        st1.angular_velocity -= m_iinertia1 * angimpulse;
    }
    if (m_dyn2)
    {
        state2D &st2 = state2();
        st2.angular_velocity += m_iinertia2 * angimpulse;
    }
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
            apply_linear_impulse(linimpulse);
    }
    if constexpr (AngDegrees == 1)
    {
        const float angimpulse = compute_angular_impulse(delta_impulse);
        if (!kit::approaches_zero(angimpulse))
            apply_angular_impulse(angimpulse);
    }
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::startup(std::vector<state2D> &states)
{
    m_states = &states;
    m_index1 = m_body1->meta.index;
    m_index2 = m_body2->meta.index;

    const state2D &st1 = state1();
    m_imass1 = st1.inv_mass();
    m_iinertia1 = st1.inv_inertia();
    m_dyn1 = st1.is_dynamic();

    const state2D &st2 = state2();
    m_imass2 = st2.inv_mass();
    m_iinertia2 = st2.inv_inertia();
    m_dyn2 = st2.is_dynamic();

    m_ts = world.integrator.ts.value;

    update_constraint_data();
    if (world.joints.constraints.params.warmup)
        warmup();
    else
        m_cumimpulse = flat_t(0.f);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
const state2D &vconstraint2D<LinDegrees, AngDegrees>::state1() const
{
    return m_states->at(m_index1);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
const state2D &vconstraint2D<LinDegrees, AngDegrees>::state2() const
{
    return m_states->at(m_index2);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
state2D &vconstraint2D<LinDegrees, AngDegrees>::state1()
{
    return m_states->at(m_index1);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
state2D &vconstraint2D<LinDegrees, AngDegrees>::state2()
{
    return m_states->at(m_index2);
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
void vconstraint2D<LinDegrees, AngDegrees>::update_constraint_data()
{
    compute_anchors_and_offsets(state1(), state2());
    if constexpr (LinDegrees == 1)
        this->m_dir = this->direction();
    m_mass = mass();
}

template <typename Mat> static Mat invert_diagonal(const Mat &mat)
{
    Mat invmat;
    for (int i = 0; i < mat.length(); ++i)
        invmat[i][i] = 1.f / mat[i][i];
    return invmat;
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
typename vconstraint2D<LinDegrees, AngDegrees>::square_t vconstraint2D<LinDegrees, AngDegrees>::mass() const
{
    if constexpr (LinDegrees + AngDegrees == 1)
    {
        const float invmass = default_inverse_mass();
        const float mass = 1.f / invmass;
        if (!m_is_soft)
            return mass;

        const auto [stiffness, damping] = spring_joint2D::stiffness_and_damping(m_frequency, m_damping_ratio, mass);
        const float igamma = m_ts * (m_ts * stiffness + damping);
        if (kit::approaches_zero(igamma))
            return mass;

        const float gamma = 1.f / igamma;
        return 1.f / (invmass + gamma);
    }
    else
    {
        const square_t invmass = default_inverse_mass();
        const square_t diagmass = invert_diagonal(invmass);
        if (!m_is_soft)
            return m_no_anchors ? diagmass : glm::inverse(invmass);

        square_t gamma{0.f};
        for (int i = 0; i < diagmass.length(); i++)
        {
            const auto [stiffness, damping] =
                spring_joint2D::stiffness_and_damping(m_frequency, m_damping_ratio, diagmass[i][i]);
            const float igamma = m_ts * (m_ts * stiffness + damping);
            if (!kit::approaches_zero(igamma))
                gamma[i][i] = 1.f / igamma;
        }
        return m_no_anchors ? invert_diagonal(invmass + gamma) : glm::inverse(invmass + gamma);
    }
}

template <std::size_t LinDegrees, std::size_t AngDegrees>
    requires LegalDegrees2D<LinDegrees, AngDegrees>
typename vconstraint2D<LinDegrees, AngDegrees>::square_t vconstraint2D<LinDegrees, AngDegrees>::default_inverse_mass()
    const
{
    if constexpr (LinDegrees == 0)
        return m_iinertia1 + m_iinertia2;
    else
    {
        const float im1 = m_imass1;
        const float im2 = m_imass2;

        const float ii1 = m_iinertia1;
        const float ii2 = m_iinertia2;

        if (m_no_anchors)
        {
            if constexpr (LinDegrees == 1 && AngDegrees == 0)
                return im1 + im2;
            else if constexpr (LinDegrees == 1 && AngDegrees == 1)
                return glm::mat2{{im1 + im2, 0.f}, {0.f, ii1 + ii2}};
            else if constexpr (LinDegrees == 2 && AngDegrees == 0)
                return glm::mat2{{im1 + im2, 0.f}, {0.f, im1 + im2}};
            else if constexpr (LinDegrees == 2 && AngDegrees == 1)
                return glm::mat3{{im1 + im2, 0.f, 0.f}, {0.f, im1 + im2, 0.f}, {0.f, 0.f, ii1 + ii2}};
        }

        if constexpr (LinDegrees == 1)
        {
            const float cross1 = kit::cross2D(m_offset1, this->m_dir);
            const float cross2 = kit::cross2D(m_offset2, this->m_dir);
            const float diag1 = im1 + im2 + ii1 * cross1 * cross1 + ii2 * cross2 * cross2;
            if constexpr (AngDegrees == 0)
                return diag1;
            else
            {
                const float cross_term = ii1 * cross1 + ii2 * cross2;
                return glm::mat2{{diag1, cross_term}, {cross_term, ii1 + ii2}};
            }
        }
        else if constexpr (LinDegrees == 2)
        {
            const glm::vec2 a1 = m_offset1 * m_offset1;
            const glm::vec2 a2 = m_offset2 * m_offset2;
            const glm::vec2 a12 = {m_offset1.x * m_offset1.y, m_offset2.x * m_offset2.y};

            const float diag1 = im1 + im2 + ii1 * a1.y + ii2 * a2.y;
            const float diag2 = im1 + im2 + ii1 * a1.x + ii2 * a2.x;

            const float cross_term1 = -ii1 * a12.x - ii2 * a12.y;
            if constexpr (AngDegrees == 0)
                return glm::mat2{{diag1, cross_term1}, {cross_term1, diag2}};
            else
            {
                const float cross_term2 = -ii1 * m_offset1.y - ii2 * m_offset2.y;
                const float cross_term3 = ii1 * m_offset1.x + ii2 * m_offset2.x;
                return glm::mat3{{diag1, cross_term1, cross_term2},
                                 {cross_term1, diag2, cross_term3},
                                 {cross_term2, cross_term3, ii1 + ii2}};
            }
        }
    }
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

template class vconstraint2D<1, 0>;
template class vconstraint2D<0, 1>;
template class vconstraint2D<1, 1>;
template class vconstraint2D<2, 0>;
template class vconstraint2D<2, 1>;

} // namespace ppx