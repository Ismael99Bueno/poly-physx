#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/nonpen_contact2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
nonpen_contact2D::nonpen_contact2D(world2D &world, const collision2D *collision, const std::size_t manifold_index)
    : joint2D(world, collision->collider1->body(), collision->collider2->body(),
              collision->manifold[manifold_index].point),
      contact2D(collision, manifold_index), m_friction_contact(world, collision, m_normal, manifold_index),
      m_has_friction(!kit::approaches_zero(collision->friction))
{
    m_ganchor1 = collision->manifold[manifold_index].point;
    if (!kit::approaches_zero(m_restitution))
    {
        m_init_ctr_vel =
            glm::dot(m_normal, m_body2->state().gvelocity_at(m_ganchor1) - m_body1->state().gvelocity_at(m_ganchor1));
        if (glm::abs(m_init_ctr_vel) < 6.f)
            m_init_ctr_vel = 0.f;
    }
}

float nonpen_contact2D::constraint_position() const
{
    return m_point.penetration + m_pntr_correction;
}
float nonpen_contact2D::constraint_velocity() const
{
    return m_restitution * m_init_ctr_vel + glm::dot(m_dir, state2().velocity_at_centroid_offset(m_offset2) -
                                                                state1().velocity_at_centroid_offset(m_offset1));
}

void nonpen_contact2D::solve_velocities()
{
    if (m_has_friction)
        m_friction_contact.solve_velocities();
    solve_velocities_clamped(0.f, FLT_MAX);
}

void nonpen_contact2D::startup(std::vector<state2D> &states)
{
    m_friction_contact.max_impulse = m_cumimpulse;
    pvconstraint2D<1, 0>::startup(states);

    if (!m_is_adjusting_positions && m_has_friction)
        m_friction_contact.startup(states);
}

void nonpen_contact2D::update_position_data()
{
    m_is_adjusting_positions = true;
    const glm::vec2 g1 = m_ganchor1;
    pvconstraint2D::update_position_data();
    const glm::vec2 dpos1 = m_ganchor1 - g1;
    m_pntr_correction = std::abs(glm::dot(m_dir, dpos1));
}

void nonpen_contact2D::warmup()
{
    pvconstraint2D::warmup();
    if (m_has_friction)
        m_friction_contact.warmup();
}

void nonpen_contact2D::update(const collision2D *collision, const std::size_t manifold_index)
{
    contact2D::update(collision, manifold_index);
    KIT_ASSERT_ERROR(collision->friction >= 0.f, "Friction must be non-negative: {0}", collision->friction)
    KIT_ASSERT_ERROR(collision->restitution >= 0.f, "Restitution must be non-negative: {0}", collision->restitution)
    m_lanchor1 =
        m_body1->state().local_position_point(collision->manifold[manifold_index].point); // lanchor2 is not used
    m_friction_contact.update(collision, m_lanchor1, m_normal);
    m_is_adjusting_positions = false;
    m_pntr_correction = 0.f;
    m_has_friction = !kit::approaches_zero(collision->friction);

    m_is_soft = global_props.is_soft;
    m_frequency = global_props.frequency;
    m_damping_ratio = global_props.damping_ratio;
}

glm::vec2 nonpen_contact2D::direction() const
{
    return m_normal;
}

} // namespace ppx
