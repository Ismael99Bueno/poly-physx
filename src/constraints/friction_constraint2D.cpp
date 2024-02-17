#include "ppx/internal/pch.hpp"
#include "ppx/constraints/friction_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
friction_constraint2D::friction_constraint2D(world2D &world, const collision2D *collision,
                                             const std::size_t manifold_index)
    : joint_constraint2D(world, "Friction", false), m_body1(collision->collider1->parent().raw()),
      m_body2(collision->collider2->parent().raw()), m_anchor1(collision->touch1(manifold_index) - m_body1->centroid()),
      m_anchor2(collision->touch2(manifold_index) - m_body2->centroid()),
      m_tangent(glm::normalize(glm::vec2(-collision->mtv.y, collision->mtv.x))), m_friction(collision->friction)
{
}

float friction_constraint2D::constraint_value() const
{
    return 0.f;
}
float friction_constraint2D::constraint_velocity() const
{
    return glm::dot(m_tangent, m_body2->ctr_proxy.velocity_at(m_anchor2) - m_body1->ctr_proxy.velocity_at(m_anchor1));
}

void friction_constraint2D::warmup()
{
    apply_warmup(*m_body1, *m_body2, m_anchor1, m_anchor2, m_tangent);
}
void friction_constraint2D::solve()
{
    const float mu = m_friction * max_lambda;
    solve_clamped(*m_body1, *m_body2, m_anchor1, m_anchor2, m_tangent, -mu, mu);
}

void friction_constraint2D::update(const collision2D *collision, const glm::vec2 &normal, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2)
{
    m_body1 = collision->collider1->parent().raw();
    m_body2 = collision->collider2->parent().raw();
    m_friction = collision->friction;
    m_anchor1 = anchor1;
    m_anchor2 = anchor2;
    m_tangent = glm::vec2(-normal.y, normal.x);
}

bool friction_constraint2D::contains(kit::uuid id) const
{
    return m_body1->id == id || m_body2->id == id;
}
bool friction_constraint2D::valid() const
{
    return m_body1 && m_body2;
}

} // namespace ppx