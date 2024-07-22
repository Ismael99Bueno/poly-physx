#include "ppx/internal/pch.hpp"
#include "ppx/collision/contacts/nonpen_friction2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
nonpen_friction2D::nonpen_friction2D(world2D &world, const collision2D *collision, const glm::vec2 &normal,
                                     const std::size_t manifold_index)
    : joint2D(world, collision->collider1->body(), collision->collider2->body(),
              collision->manifold[manifold_index].point),
      m_friction(collision->friction)
{
    m_tangent = glm::vec2(-normal.y, normal.x);
}
float nonpen_friction2D::constraint_velocity() const
{
    return glm::dot(m_dir,
                    state2().velocity_at_centroid_offset(m_offset2) - state1().velocity_at_centroid_offset(m_offset1));
}

void nonpen_friction2D::solve_velocities()
{
    const float mu = m_friction * max_impulse;
    solve_velocities_clamped(-mu, mu);
}

void nonpen_friction2D::update(const collision2D *collision, const glm::vec2 &lanchor1, const glm::vec2 &lanchor2,
                               const glm::vec2 &normal)
{
    m_body1 = collision->collider1->body();
    m_body2 = collision->collider2->body();
    m_friction = collision->friction;
    m_lanchor1 = lanchor1;
    m_lanchor2 = lanchor2;
    m_tangent = glm::vec2(-normal.y, normal.x);

    m_is_soft = global_props.is_soft;
    m_frequency = global_props.frequency;
    m_damping_ratio = global_props.damping_ratio;
}

glm::vec2 nonpen_friction2D::direction() const
{
    return m_tangent;
}

} // namespace ppx