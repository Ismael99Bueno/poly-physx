#include "ppx/internal/pch.hpp"
#include "ppx/constraints/friction_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
friction_constraint2D::friction_constraint2D(world2D &world, const collision2D *collision,
                                             const std::size_t manifold_index)
    : vconstraint2D(world, collision->collider1->body(), collision->collider2->body(),
                    collision->manifold[manifold_index].point),
      m_friction(collision->friction), m_mtv(glm::normalize(collision->mtv))
{
}
float friction_constraint2D::constraint_velocity() const
{
    return glm::dot(m_dir, m_body2->ctr_state.velocity_at_centroid_offset(m_offset2) -
                               m_body1->ctr_state.velocity_at_centroid_offset(m_offset1));
}

void friction_constraint2D::solve()
{
    const float mu = m_friction * max_impulse;
    solve_clamped(-mu, mu);
}

void friction_constraint2D::update(const collision2D *collision, const glm::vec2 &lanchor1, const glm::vec2 &norm_mtv)
{
    m_body1 = collision->collider1->body();
    m_body2 = collision->collider2->body();
    m_friction = collision->friction;
    m_lanchor1 = lanchor1;
    m_mtv = norm_mtv;
}

float friction_constraint2D::inverse_mass() const
{
    const float cross1 = kit::cross2D(m_offset1, m_dir);
    const float cross2 = kit::cross2D(m_offset2, m_dir);
    return m_body1->props().dynamic.inv_mass + m_body2->props().dynamic.inv_mass +
           m_body1->props().dynamic.inv_inertia * cross1 * cross1 +
           m_body2->props().dynamic.inv_inertia * cross2 * cross2;
}

glm::vec2 friction_constraint2D::direction() const
{
    return glm::vec2(-m_mtv.y, m_mtv.x);
}

} // namespace ppx