#include "ppx/internal/pch.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
contact_constraint2D::contact_constraint2D(world2D &world, const collision2D *collision,
                                           const std::size_t manifold_index, const float slop)
    : joint_constraint2D(world, "Contact"), m_body1(&collision->collider1->parent()),
      m_body2(&collision->collider2->parent()), m_anchor1(collision->touch1(manifold_index) - m_body1->centroid()),
      m_anchor2(collision->touch2(manifold_index) - m_body2->centroid()), m_normal(glm::normalize(collision->mtv)),
      m_index(manifold_index), m_restitution(collision->restitution), m_slop(slop),
      m_penetration(-glm::length(collision->mtv)), m_friction(world, collision, manifold_index)
{
    KIT_ASSERT_ERROR(collision->friction >= 0.f, "Friction must be non-negative: {0}", collision->friction)
    KIT_ASSERT_ERROR(collision->restitution >= 0.f, "Restitution must be non-negative: {0}", collision->restitution)
    m_init_ctr_vel = constraint_velocity();
}

float contact_constraint2D::constraint_value() const
{
    return m_penetration;
}
float contact_constraint2D::constraint_velocity() const
{
    const float cdot =
        m_restitution * m_init_ctr_vel +
        glm::dot(m_normal, m_body2->ctr_proxy.velocity_at(m_anchor2) - m_body1->ctr_proxy.velocity_at(m_anchor1));
    const float abs_cdot = std::abs(cdot);
    if (abs_cdot < m_slop)
        return cdot * abs_cdot / m_slop;
    return cdot;
}

void contact_constraint2D::warmup()
{
    apply_warmup(*m_body1, *m_body2, m_anchor1, m_anchor2, m_normal);
    m_friction.warmup();
}
void contact_constraint2D::solve()
{
    solve_clamped(*m_body1, *m_body2, m_anchor1, m_anchor2, m_normal, 0.f, FLT_MAX);
    m_friction.max_lambda = std::abs(m_accumulated_lambda);
    m_friction.solve();
}

void contact_constraint2D::update(const collision2D *collision, const float slop)
{
    KIT_ASSERT_ERROR(collision->friction >= 0.f, "Friction must be non-negative: {0}", collision->friction)
    KIT_ASSERT_ERROR(collision->restitution >= 0.f, "Restitution must be non-negative: {0}", collision->restitution)
    m_body1 = &collision->collider1->parent();
    m_body2 = &collision->collider2->parent();
    m_anchor1 = collision->touch1(m_index) - collision->collider1->gcentroid();
    m_anchor2 = collision->touch2(m_index) - collision->collider2->gcentroid();
    m_normal = glm::normalize(collision->mtv);
    m_restitution = collision->restitution;
    m_slop = slop;
    m_friction.update(collision, m_normal, m_anchor1, m_anchor2);
    recently_updated = true;
}

bool contact_constraint2D::contains(kit::uuid id) const
{
    return m_body1->id == id || m_body2->id == id;
}

bool contact_constraint2D::valid() const
{
    return m_body1 && m_body2;
}

} // namespace ppx
