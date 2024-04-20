#include "ppx/internal/pch.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
contact_constraint2D::contact_constraint2D(world2D &world, const collision2D *collision,
                                           const std::size_t manifold_index)
    : pvconstraint2D(world, collision->collider1->body(), collision->collider2->body(),
                     collision->manifold[manifold_index].point),
      m_restitution(collision->restitution), m_penetration(collision->manifold[manifold_index].penetration),
      m_nmtv(glm::normalize(collision->mtv)), m_friction(world, collision, m_nmtv, manifold_index),
      m_has_friction(!kit::approaches_zero(collision->friction))
{
    m_ganchor1 = collision->manifold[manifold_index].point;
    m_use_both_anchors = false;
    if (!kit::approaches_zero(m_restitution))
        m_init_ctr_vel = glm::dot(m_nmtv, m_body2->gvelocity_at(m_ganchor1) - m_body1->gvelocity_at(m_ganchor1));
}

float contact_constraint2D::constraint_position() const
{
    return m_penetration + m_pntr_correction;
}
float contact_constraint2D::constraint_velocity() const
{
    return m_restitution * m_init_ctr_vel +
           glm::dot(m_dir, m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
                               m_body1->ctr_state.velocity_at_centroid_offset(m_offset1));
}

void contact_constraint2D::solve_velocities()
{
    m_friction.max_impulse = m_cumimpulse;
    if (m_has_friction)
        m_friction.solve_velocities();
    solve_velocities_clamped(0.f, FLT_MAX);
}

void contact_constraint2D::startup()
{
    pvconstraint2D<1, 0>::startup();
    if (!m_is_adjusting_positions && m_has_friction)
        m_friction.startup();
}

void contact_constraint2D::update_position_data()
{
    m_is_adjusting_positions = true;
    const glm::vec2 g1 = m_ganchor1;
    pvconstraint2D::update_position_data();
    const glm::vec2 dpos1 = m_ganchor1 - g1;
    m_pntr_correction = std::abs(glm::dot(m_dir, dpos1));
}

void contact_constraint2D::warmup()
{
    pvconstraint2D::warmup();
    if (m_has_friction)
        m_friction.warmup();
}

void contact_constraint2D::update(const collision2D *collision, const std::size_t manifold_index)
{
    KIT_ASSERT_ERROR(collision->friction >= 0.f, "Friction must be non-negative: {0}", collision->friction)
    KIT_ASSERT_ERROR(collision->restitution >= 0.f, "Restitution must be non-negative: {0}", collision->restitution)
    m_body1 = collision->collider1->body();
    m_body2 = collision->collider2->body();
    m_lanchor1 = m_body1->local_position_point(collision->manifold[manifold_index].point); // lanchor2 is not used
    m_nmtv = glm::normalize(collision->mtv);
    m_restitution = collision->restitution;
    m_penetration = -glm::length(collision->mtv);
    m_friction.update(collision, m_lanchor1, m_nmtv);
    recently_updated = true;
    m_is_adjusting_positions = false;
    m_pntr_correction = 0.f;
    m_has_friction = !kit::approaches_zero(collision->friction);
}

glm::vec2 contact_constraint2D::direction() const
{
    return m_nmtv;
}

} // namespace ppx
