#include "ppx/internal/pch.hpp"
#include "ppx/constraints/friction_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
friction_constraint2D::friction_constraint2D(world2D &world, const collision2D *collision, const glm::vec2 &nmtv,
                                             const std::size_t manifold_index)
    : vconstraint10_2D(world, collision->collider1->body(), collision->collider2->body(),
                       collision->manifold[manifold_index].point),
      m_friction(collision->friction)
{
    m_tangent = glm::vec2(-nmtv.y, nmtv.x);
    m_use_both_anchors = false;
}
float friction_constraint2D::constraint_velocity() const
{
    return glm::dot(m_dir, m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
                               m_body1->ctr_state.velocity_at_centroid_offset(m_offset1));
}

void friction_constraint2D::solve_velocities()
{
    const float mu = m_friction * max_impulse;
    solve_velocities_clamped(-mu, mu);
}

void friction_constraint2D::update(const collision2D *collision, const glm::vec2 &lanchor1, const glm::vec2 &nmtv)
{
    m_body1 = collision->collider1->body();
    m_body2 = collision->collider2->body();
    m_friction = collision->friction;
    m_lanchor1 = lanchor1;
    m_tangent = glm::vec2(-nmtv.y, nmtv.x);
}

glm::vec2 friction_constraint2D::direction() const
{
    return m_tangent;
}

} // namespace ppx