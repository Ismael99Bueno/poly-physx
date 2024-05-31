#include "ppx/internal/pch.hpp"
#include "ppx/joints/spring_joint2D.hpp"
#include "ppx/world2D.hpp"
#ifdef KIT_USE_YAML_CPP
#include "ppx/serialization/serialization.hpp"
#endif
#include "kit/utility/utils.hpp"

namespace ppx
{
spring_joint2D::spring_joint2D(world2D &world, const specs &spc)
    : joint2D(world, spc, spc.ganchor1, spc.ganchor2), m_props(spc.props)
{
    if (spc.deduce_length)
    {
        const float length = glm::distance(spc.ganchor1, spc.ganchor2);
        m_props.min_length = length;
        m_props.max_length = length;
    }
}

glm::vec2 spring_joint2D::non_linear_displacement(const glm::vec2 &displacement) const
{
    glm::vec2 nl_term = displacement;
    glm::vec2 nl_cummulative = displacement;
    float decay = 16.f;
    for (std::uint32_t term = 0; term < m_props.non_linear_terms; term++)
    {
        nl_term *= displacement * displacement;
        nl_cummulative += m_props.non_linear_contribution * nl_term / decay;
        decay *= decay;
    }
    return nl_cummulative;
}

glm::vec3 spring_joint2D::compute_force() const
{
    const glm::vec2 relpos = m_ganchor2 - m_ganchor1;
    const float length = glm::length(relpos);
    if (length >= m_props.min_length && length <= m_props.max_length)
        return glm::vec3(0.f);
    const float efflength = length < m_props.min_length ? m_props.min_length : m_props.max_length;

    const glm::vec2 direction =
        !kit::approaches_zero(glm::length2(relpos)) ? glm::normalize(relpos) : glm::vec2(1.f, 0.f);
    const glm::vec2 relvel = direction * glm::dot(m_body2->velocity_at_centroid_offset(m_offset2) -
                                                      m_body1->velocity_at_centroid_offset(m_offset1),
                                                  direction);
    const glm::vec2 vlen = efflength * direction;
    const glm::vec2 displacement = relpos - vlen;

    const float meff = 1.f / (m_body1->props().dynamic.inv_mass + m_body2->props().dynamic.inv_mass);
    const auto [stiffness, damping] = stiffness_and_damping(m_props.frequency, m_props.damping_ratio, meff);

    const glm::vec2 force =
        -(stiffness * (m_props.non_linear_terms != 0 ? non_linear_displacement(displacement) : displacement) +
          damping * relvel);
    return glm::vec3(force, 0.f);
}

float spring_joint2D::kinetic_energy() const
{
    return m_body1->kinetic_energy() + m_body2->kinetic_energy();
}
float spring_joint2D::potential_energy() const
{
    const glm::vec2 ga1 = ganchor1(); // anchors may have changed by user
    const glm::vec2 ga2 = ganchor2();

    const float length = glm::distance(ga1, ga2);
    if (length >= m_props.min_length && length <= m_props.max_length)
        return 0.f;
    const float dist = length < m_props.min_length ? m_props.min_length - length : length - m_props.max_length;
    return 0.5f * m_props.frequency * dist * dist;
}
float spring_joint2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

const spring_joint2D::specs::properties &spring_joint2D::props() const
{
    return m_props;
}
void spring_joint2D::props(const specs::properties &props)
{
    KIT_ASSERT_ERROR(props.frequency >= 0.f, "Frequency must be non-negative: {0}", props.frequency)
    KIT_ASSERT_ERROR(props.damping_ratio >= 0.f, "Damping ratio must be non-negative: {0}", props.damping_ratio)
    KIT_ASSERT_ERROR(props.min_length <= props.max_length,
                     "Minimum length must be less than or equal to maximum length")
    KIT_ASSERT_ERROR(props.non_linear_contribution >= 0.f, "Non linear contribution must be non-negative: {0}",
                     props.non_linear_contribution)
    m_props = props;
    awake();
}

std::pair<float, float> spring_joint2D::frequency_and_ratio(float stiffness, float damping, float mass)
{
    KIT_ASSERT_ERROR(stiffness >= 0.f, "Stiffness must be non-negative: {0}", stiffness)
    KIT_ASSERT_ERROR(damping >= 0.f, "Damping must be non-negative: {0}", damping)
    KIT_ASSERT_ERROR(mass > 0.f, "Mass must be positive: {0}", mass)

    const float frequency = glm::sqrt(stiffness / mass) / (2.f * glm::pi<float>());
    const float damping_ratio = damping / (4.f * glm::pi<float>() * mass * frequency);
    return {frequency, damping_ratio};
}

std::pair<float, float> spring_joint2D::stiffness_and_damping(float frequency, float damping_ratio, float mass)
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