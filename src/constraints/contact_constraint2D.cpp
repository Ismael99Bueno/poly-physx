#include "ppx/internal/pch.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
contact_constraint2D::contact_constraint2D(world2D &world, const collision2D *collision,
                                           const std::size_t manifold_index)
    : pvconstraint2D(world, collision->collider1->body(), collision->collider2->body(),
                     collision->manifold.contacts[manifold_index]),
      m_manifold_index(manifold_index), m_restitution(collision->restitution),
      m_penetration(-glm::length(collision->mtv)), m_mtv(collision->mtv), m_friction(world, collision, manifold_index)
{
    m_init_ctr_vel = glm::dot(glm::normalize(collision->mtv),
                              m_body2->gvelocity_at(collision->manifold.contacts[manifold_index]) -
                                  m_body1->gvelocity_at(collision->manifold.contacts[manifold_index]));
}

float contact_constraint2D::constraint_position() const
{
    return m_penetration;
}
float contact_constraint2D::constraint_velocity() const
{
    return m_restitution * m_init_ctr_vel +
           glm::dot(m_dir, m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
                               m_body1->ctr_state.velocity_at_centroid_offset(m_offset1));
}

void contact_constraint2D::solve()
{
    m_friction.max_lambda = std::abs(m_cumlambda);
    m_friction.solve();
    solve_clamped(0.f, FLT_MAX);
}

void contact_constraint2D::startup()
{
    pvconstraint2D::startup();
    if (!m_is_adjusting_positions)
        m_friction.startup();
}

void contact_constraint2D::warmup()
{
    pvconstraint2D::warmup();
    m_friction.warmup();
}

void contact_constraint2D::update(const collision2D *collision)
{
    KIT_ASSERT_ERROR(collision->friction >= 0.f, "Friction must be non-negative: {0}", collision->friction)
    KIT_ASSERT_ERROR(collision->restitution >= 0.f, "Restitution must be non-negative: {0}", collision->restitution)
    m_body1 = collision->collider1->body();
    m_body2 = collision->collider2->body();
    m_lanchor1 = m_body1->local_position_point(collision->manifold.contacts[m_manifold_index]);
    m_lanchor2 = m_body2->local_position_point(collision->manifold.contacts[m_manifold_index]);
    m_mtv = collision->mtv;
    m_restitution = collision->restitution;
    m_penetration = -glm::length(collision->mtv);
    m_friction.update(collision, m_lanchor1, m_lanchor2);
    recently_updated = true;
    m_is_adjusting_positions = false;
}

float contact_constraint2D::inverse_mass() const
{
    const float cross1 = kit::cross2D(m_offset1, m_dir);
    const float cross2 = kit::cross2D(m_offset2, m_dir);
    return m_body1->props().dynamic.inv_mass + m_body2->props().dynamic.inv_mass +
           m_body1->props().dynamic.inv_inertia * cross1 * cross1 +
           m_body2->props().dynamic.inv_inertia * cross2 * cross2;
}

glm::vec2 contact_constraint2D::direction() const
{
    return glm::normalize(m_mtv);
}

} // namespace ppx
