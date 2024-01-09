#include "ppx/internal/pch.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
contact_constraint2D::contact_constraint2D(const collision2D *collision, const std::size_t manifold_index,
                                           const float restitution, const float friction, const float slop)
    : joint_constraint2D("Contact"), m_collision(collision),
      m_anchor1(collision->touch1(manifold_index) - collision->body1->position()),
      m_anchor2(collision->touch2(manifold_index) - collision->body2->position()),
      m_normal(glm::normalize(collision->mtv)), m_index(manifold_index), m_restitution(restitution), m_slop(slop),
      m_friction(collision, manifold_index, friction)
{
    m_init_ctr_vel = constraint_velocity();
}

void contact_constraint2D::set_world(world2D *world)
{
    this->world = world;
    m_friction.world = world;
}

float contact_constraint2D::constraint_value() const
{
    return glm::dot(m_collision->touch2(m_index) - m_collision->touch1(m_index), m_normal);
}
float contact_constraint2D::constraint_velocity() const
{
    const float cdot =
        m_restitution * m_init_ctr_vel + glm::dot(m_normal, m_collision->body2->constraint_velocity_at(m_anchor2) -
                                                                m_collision->body1->constraint_velocity_at(m_anchor1));
    const float abs_cdot = std::abs(cdot);
    if (abs_cdot < m_slop)
        return cdot * abs_cdot / m_slop;
    return cdot;
}

void contact_constraint2D::warmup()
{
    apply_warmup(*m_collision->body1, *m_collision->body2, m_anchor1, m_anchor2, m_normal);
    m_friction.warmup();
}
void contact_constraint2D::solve()
{
    solve_clamped(*m_collision->body1, *m_collision->body2, m_anchor1, m_anchor2, m_normal, 0.f, FLT_MAX);
    m_friction.max_lambda = std::abs(m_accumulated_lambda);
    m_friction.solve();
}

void contact_constraint2D::update(const collision2D *collision, const float restitution, const float friction,
                                  const float slop)
{
    m_collision = collision;
    m_anchor1 = collision->touch1(m_index) - collision->body1->position();
    m_anchor2 = collision->touch2(m_index) - collision->body2->position();
    m_normal = glm::normalize(collision->mtv);
    m_restitution = restitution;
    m_slop = slop;
    m_friction.update(collision, m_normal, m_anchor1, m_anchor2, friction);
    recently_updated = true;
}

bool contact_constraint2D::contains(kit::uuid id) const
{
    return m_collision->body1->id == id || m_collision->body2->id == id;
}

bool contact_constraint2D::valid() const
{
    return m_collision->body1 && m_collision->body2;
}

} // namespace ppx
