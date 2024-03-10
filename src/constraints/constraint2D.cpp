#include "ppx/internal/pch.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/serialization/serialization.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
constraint2D::constraint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2,
                           const glm::vec2 &ganchor1, const glm::vec2 &ganchor2, const bool m_allow_baumgarte)
    : joint2D(world, body1, body2, ganchor1, ganchor2), worldref2D(world), m_allow_baumgarte(m_allow_baumgarte)
{
}

constraint2D::constraint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &ganchor,
                           const bool m_allow_baumgarte)
    : joint2D(world, body1, body2, ganchor), worldref2D(world), m_allow_baumgarte(m_allow_baumgarte)
{
}

float constraint2D::compute_lambda() const
{
    const float cvel = constraint_velocity();
    const float inv_mass = inverse_mass();

    if (m_allow_baumgarte && world.constraints.baumgarte_correction)
    {
        const float c = constraint_value();
        if (std::abs(c) > world.constraints.baumgarte_threshold)
            return -(cvel + world.constraints.baumgarte_coef * c / world.integrator.ts.value) / inv_mass;
    }
    return -cvel / inv_mass;
}
void constraint2D::apply_lambda(const float lambda)
{
    const glm::vec2 imp2 = lambda * m_dir;
    const glm::vec2 imp1 = -imp2;

    m_body1->ctr_proxy.velocity += m_body1->props().dynamic.inv_mass * imp1;
    m_body2->ctr_proxy.velocity += m_body2->props().dynamic.inv_mass * imp2;

    m_body1->ctr_proxy.angular_velocity += m_body1->props().dynamic.inv_inertia * kit::cross2D(m_offset1, imp1);
    m_body2->ctr_proxy.angular_velocity += m_body2->props().dynamic.inv_inertia * kit::cross2D(m_offset2, imp2);

    const glm::vec2 f1 = imp1 / world.integrator.ts.value;
    const glm::vec2 f2 = imp2 / world.integrator.ts.value;
    const float torque1 = kit::cross2D(m_offset1, f1);
    const float torque2 = kit::cross2D(m_offset2, f2);

    m_body1->apply_simulation_force(f1);
    m_body1->apply_simulation_torque(torque1);
    m_body2->apply_simulation_force(f2);
    m_body2->apply_simulation_torque(torque2);
}

void constraint2D::solve_clamped(const float min, const float max)
{
    const float lambda = compute_lambda();
    const float old_lambda = m_cumlambda;
    m_cumlambda = std::clamp(m_cumlambda + lambda, min, max);

    const float delta_lambda = m_cumlambda - old_lambda;
    if (!kit::approaches_zero(delta_lambda))
        apply_lambda(delta_lambda);
}

void constraint2D::solve_unclamped()
{
    const float lambda = compute_lambda();
    m_cumlambda += lambda;
    apply_lambda(lambda);
}

void constraint2D::startup()
{
    m_offset1 = ganchor1() - m_body1->centroid();
    m_offset2 = ganchor2() - m_body2->centroid();
    m_dir = direction();
}

void constraint2D::warmup()
{
    if (kit::approaches_zero(m_cumlambda))
        return;
    m_cumlambda *= world.timestep_ratio();
    apply_lambda(m_cumlambda);
}

} // namespace ppx