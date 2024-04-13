#include "ppx/internal/pch.hpp"
#include "ppx/joints/spring2D.hpp"
#include "ppx/world2D.hpp"
#ifdef KIT_USE_YAML_CPP
#include "ppx/serialization/serialization.hpp"
#endif
#include "kit/utility/utils.hpp"

namespace ppx
{

spring2D::spring2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.ganchor1, spc.ganchor2), frequency(spc.props.frequency),
      damping_ratio(spc.props.damping_ratio),
      length(spc.props.deduce_length ? spc.props.length : glm::distance(spc.ganchor1, spc.ganchor2)),
      non_linear_terms(spc.props.non_linear_terms), non_linear_contribution(spc.props.non_linear_contribution)
{
}

glm::vec2 spring2D::non_linear_displacement(const glm::vec2 &displacement) const
{
    glm::vec2 nl_term = displacement;
    glm::vec2 nl_cummulative = displacement;
    float decay = 16.f;
    for (std::uint32_t term = 0; term < non_linear_terms; term++)
    {
        nl_term *= displacement * displacement;
        nl_cummulative += nl_term / decay;
        decay *= decay;
    }
    return nl_cummulative * non_linear_contribution;
}

glm::vec4 spring2D::force() const
{
    KIT_ASSERT_ERROR(frequency >= 0.f, "Frequency must be non-negative: {0}", frequency)
    KIT_ASSERT_ERROR(damping_ratio >= 0.f, "Damping ratio must be non-negative: {0}", damping_ratio)
    KIT_ASSERT_ERROR(length >= 0.f, "Length must be non-negative: {0}", length)
    KIT_ASSERT_ERROR(non_linear_contribution >= 0.f, "Non linear contribution must be non-negative: {0}",
                     non_linear_contribution)

    const glm::vec2 ga1 = ganchor1();
    const glm::vec2 ga2 = ganchor2();

    const glm::vec2 offset1 = ga1 - m_body1->centroid();
    const glm::vec2 offset2 = ga2 - m_body2->centroid();

    const glm::vec2 relpos = ga2 - ga1;

    const glm::vec2 direction =
        !kit::approaches_zero(glm::length2(relpos)) ? glm::normalize(relpos) : glm::vec2(1.f, 0.f);
    const glm::vec2 relvel = direction * glm::dot(m_body2->velocity_at_centroid_offset(offset2) -
                                                      m_body1->velocity_at_centroid_offset(offset1),
                                                  direction);
    const glm::vec2 vlen = length * direction;
    const glm::vec2 displacement = relpos - vlen;

    const float meff = 1.f / (m_body1->props().dynamic.inv_mass + m_body2->props().dynamic.inv_mass);
    const auto [stiffness, damping] = stiffness_and_damping(frequency, damping_ratio, meff);

    const glm::vec2 force =
        stiffness * (non_linear_terms != 0 ? non_linear_displacement(displacement) : displacement) + damping * relvel;

    const float torque1 = kit::cross2D(offset1, force);
    const float torque2 = kit::cross2D(force, offset2);
    return {force, torque1, torque2};
}

float spring2D::kinetic_energy() const
{
    return m_body1->kinetic_energy() + m_body2->kinetic_energy();
}
float spring2D::potential_energy() const
{
    const float dist = glm::distance(ganchor1(), ganchor2()) - length;
    return 0.5f * frequency * dist * dist;
}
float spring2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

void spring2D::solve()
{
    const glm::vec4 f = force();
    m_body1->apply_simulation_force(glm::vec2(f));
    m_body2->apply_simulation_force(-glm::vec2(f));

    m_body1->apply_simulation_torque(f.z);
    m_body2->apply_simulation_torque(f.w);
}

std::pair<float, float> spring2D::frequency_and_ratio(float stiffness, float damping, float mass)
{
    KIT_ASSERT_ERROR(stiffness >= 0.f, "Stiffness must be non-negative: {0}", stiffness)
    KIT_ASSERT_ERROR(damping >= 0.f, "Damping must be non-negative: {0}", damping)
    KIT_ASSERT_ERROR(mass > 0.f, "Mass must be positive: {0}", mass)

    const float frequency = glm::sqrt(stiffness / mass) / (2.f * glm::pi<float>());
    const float damping_ratio = damping / (4.f * glm::pi<float>() * mass * frequency);
    return {frequency, damping_ratio};
}

std::pair<float, float> spring2D::stiffness_and_damping(float frequency, float damping_ratio, float mass)
{
    KIT_ASSERT_ERROR(frequency >= 0.f, "Frequency must be non-negative: {0}", frequency)
    KIT_ASSERT_ERROR(damping_ratio >= 0.f, "Damping ratio must be non-negative: {0}", damping_ratio)
    KIT_ASSERT_ERROR(mass > 0.f, "Mass must be positive: {0}", mass)

    const float freq_pi_factor = 2.f * frequency * glm::pi<float>();
    const float stiffness = freq_pi_factor * freq_pi_factor * mass;
    const float damping = 2.f * damping_ratio * freq_pi_factor * mass;
    return {stiffness, damping};
}

} // namespace ppx