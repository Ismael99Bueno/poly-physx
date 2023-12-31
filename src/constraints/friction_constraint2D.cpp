#include "ppx/internal/pch.hpp"
#include "ppx/constraints/friction_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
friction_constraint2D::friction_constraint2D(const collision2D *collision, const std::size_t manifold_index,
                                             const float friction)
    : joint_constraint2D("Friction", false), m_collision(collision),
      m_anchor1(collision->touch1(manifold_index) - collision->body1->position()),
      m_anchor2(collision->touch2(manifold_index) - collision->body2->position()),
      m_tangent(glm::normalize(glm::vec2(-collision->mtv.y, collision->mtv.x))), m_friction(friction)
{
}

float friction_constraint2D::constraint_value() const
{
    return 0.f;
}
float friction_constraint2D::constraint_velocity() const
{
    return glm::dot(m_tangent, m_collision->body1->constraint_velocity_at(m_anchor1) -
                                   m_collision->body2->constraint_velocity_at(m_anchor2));
}

void friction_constraint2D::warmup()
{
    apply_warmup(*m_collision->body1, *m_collision->body2, m_anchor1, m_anchor2, m_tangent);
}
void friction_constraint2D::solve()
{
    const float mu = m_friction * max_lambda;
    solve_clamped(*m_collision->body1, *m_collision->body2, m_anchor1, m_anchor2, m_tangent, -mu, mu);
}

void friction_constraint2D::update(const collision2D *collision, const glm::vec2 &normal, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2, float friction)
{
    m_collision = collision;
    m_anchor1 = anchor1;
    m_anchor2 = anchor2;
    m_tangent = glm::vec2(-normal.y, normal.x);
    m_friction = friction;
}

bool friction_constraint2D::contains(kit::uuid id) const
{
    return m_collision->body1->id == id || m_collision->body2->id == id;
}
bool friction_constraint2D::valid() const
{
    return m_collision->body1 && m_collision->body2;
}

} // namespace ppx