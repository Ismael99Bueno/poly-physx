#include "ppx/internal/pch.hpp"
#include "ppx/constraints/contact_constraint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
contact_constraint2D::contact_constraint2D(const collision2D &collision)
    : constraint2D("Contact"), m_collision(collision), m_anchor1(collision.touch1 - collision.current->position()),
      m_anchor2(collision.touch2 - collision.incoming->position()), m_normal(glm::normalize(collision.normal))
{
}

float contact_constraint2D::constraint_value() const
{
    return glm::dot(m_collision.touch1 - m_collision.touch2, m_normal);
}
float contact_constraint2D::constraint_velocity() const
{
    return glm::dot(m_normal,
                    m_collision.current->velocity_at(m_anchor1) - m_collision.incoming->velocity_at(m_anchor2));
}
float contact_constraint2D::constraint_acceleration() const
{
    const body2D::ptr &body1 = m_collision.current;
    const body2D::ptr &body2 = m_collision.incoming;

    const glm::vec2 orth_anchor1 = glm::vec2(-m_anchor1.y, m_anchor1.x);
    const glm::vec2 orth_anchor2 = glm::vec2(-m_anchor2.y, m_anchor2.x);

    const glm::vec2 lin_term1 = body1->inv_mass() * body1->force() +
                                body1->inv_inertia() * body1->torque() * orth_anchor1 -
                                m_anchor1 * body1->angular_velocity() * body1->angular_velocity();
    const glm::vec2 lin_term2 = body2->inv_mass() * body2->force() +
                                body2->inv_inertia() * body2->torque() * orth_anchor2 -
                                m_anchor2 * body2->angular_velocity() * body2->angular_velocity();
    const float cross = kit::cross2D(m_normal, body1->velocity() - body2->velocity());

    const float lin_term = glm::dot(m_normal, lin_term1 - lin_term2);
    const float ang_term = cross * cross / glm::max(1.f, glm::distance(m_collision.touch1, m_collision.touch2));
    return lin_term + ang_term;
}

float contact_constraint2D::compute_lambda() const
{
    static constexpr float stiffness = 1200.f;
    static constexpr float dampening = 10.f;
    const float ts = world->current_timestep();

    const float c = constraint_value();
    const float cvel = constraint_velocity() * (1.f + ts * dampening) + c * ts * stiffness;

    // const float cacc = constraint_acceleration() + c * stiffness + cvel * dampening;

    const float cross1 = kit::cross2D(m_anchor1, m_normal);
    const float cross2 = kit::cross2D(m_anchor2, m_normal);

    const body2D::ptr &body1 = m_collision.current;
    const body2D::ptr &body2 = m_collision.incoming;

    const float inv_mass = body1->inv_mass() + body2->inv_mass() + body1->inv_inertia() * cross1 * cross1 +
                           body2->inv_inertia() * cross2 * cross2;

    return -cvel / inv_mass;
}

void contact_constraint2D::apply_lambda(const float lambda)
{
    const glm::vec2 f1 = lambda * m_normal;
    const glm::vec2 f2 = -f1;

    m_collision.current->boost(f1 * m_collision.current->inv_mass());
    m_collision.current->spin(kit::cross2D(m_anchor1, f1) * m_collision.current->inv_inertia());

    m_collision.incoming->boost(f2 * m_collision.incoming->inv_mass());
    m_collision.incoming->spin(kit::cross2D(m_anchor2, f2) * m_collision.incoming->inv_inertia());
}

void contact_constraint2D::warmup()
{
    if (kit::approaches_zero(m_accumulated_lambda))
        return;
    m_accumulated_lambda *= world->timestep_ratio();
    apply_lambda(m_accumulated_lambda);
}
void contact_constraint2D::solve()
{
    const float lambda = compute_lambda();
    const float old_lambda = m_accumulated_lambda;

    m_accumulated_lambda += lambda;
    m_accumulated_lambda = std::min(0.f, m_accumulated_lambda);

    const float delta_lambda = m_accumulated_lambda - old_lambda;
    if (!kit::approaches_zero(delta_lambda))
        apply_lambda(delta_lambda);
}

bool contact_constraint2D::contains(kit::uuid id) const
{
    return m_collision.current->id == id || m_collision.incoming->id == id;
}

bool contact_constraint2D::valid() const
{
    return m_collision.current && m_collision.incoming;
}

} // namespace ppx
