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
    : joint2D(world, spc, spc.ganchor1, spc.ganchor2), m_frequency(spc.props.frequency),
      m_damping_ratio(spc.props.damping_ratio), m_min_length(spc.props.min_length), m_max_length(spc.props.max_length),
      m_non_linear_terms(spc.props.non_linear_terms), m_non_linear_contribution(spc.props.non_linear_contribution)
{
    if (spc.deduce_length)
    {
        const float length = glm::distance(spc.ganchor1, spc.ganchor2);
        m_min_length = length;
        m_max_length = length;
    }
}

glm::vec2 spring_joint2D::non_linear_displacement(const glm::vec2 &displacement) const
{
    glm::vec2 nl_term = displacement;
    glm::vec2 nl_cummulative = displacement;
    float decay = 16.f;
    for (std::uint32_t term = 0; term < m_non_linear_terms; term++)
    {
        nl_term *= displacement * displacement;
        nl_cummulative += m_non_linear_contribution * nl_term / decay;
        decay *= decay;
    }
    return nl_cummulative;
}

glm::vec3 spring_joint2D::compute_force(const state2D &state1, const state2D &state2) const
{
    const glm::vec2 relpos = m_ganchor2 - m_ganchor1;
    const float length = glm::length(relpos);
    if (length >= m_min_length && length <= m_max_length)
        return glm::vec3(0.f);

    const glm::vec2 vel1 = state1.velocity_at_centroid_offset(m_offset1);
    const glm::vec2 vel2 = state2.velocity_at_centroid_offset(m_offset2);

    const float imass1 = state1.inv_mass();
    const float imass2 = state2.inv_mass();

    const float efflength = length < m_min_length ? m_min_length : m_max_length;

    const glm::vec2 direction =
        !kit::approaches_zero(glm::length2(relpos)) ? glm::normalize(relpos) : glm::vec2(1.f, 0.f);
    const glm::vec2 relvel = direction * glm::dot(vel2 - vel1, direction);
    const glm::vec2 vlen = efflength * direction;
    const glm::vec2 displacement = relpos - vlen;

    const float meff = 1.f / (imass1 + imass2);
    const auto [stiffness, damping] = stiffness_and_damping(m_frequency, m_damping_ratio, meff);

    const glm::vec2 force =
        -(stiffness * (m_non_linear_terms != 0 ? non_linear_displacement(displacement) : displacement) +
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
    if (length >= m_min_length && length <= m_max_length)
        return 0.f;
    const float dist = length < m_min_length ? m_min_length - length : length - m_max_length;
    return 0.5f * m_frequency * dist * dist;
}
float spring_joint2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

spring_joint2D::specs::properties spring_joint2D::props() const
{
    specs::properties props;
    fill_jprops(props);
    props.frequency = m_frequency;
    props.damping_ratio = m_damping_ratio;
    props.min_length = m_min_length;
    props.max_length = m_max_length;
    props.non_linear_terms = m_non_linear_terms;
    props.non_linear_contribution = m_non_linear_contribution;
    return props;
}
void spring_joint2D::props(const specs::properties &props)
{
    KIT_ASSERT_ERROR(props.frequency >= 0.f, "Frequency must be non-negative: {0}", props.frequency)
    KIT_ASSERT_ERROR(props.damping_ratio >= 0.f, "Damping ratio must be non-negative: {0}", props.damping_ratio)
    KIT_ASSERT_ERROR(props.min_length <= props.max_length,
                     "Minimum length must be less than or equal to maximum length")
    KIT_ASSERT_ERROR(props.non_linear_contribution >= 0.f, "Non linear contribution must be non-negative: {0}",
                     props.non_linear_contribution)
    jprops(props);
    m_frequency = props.frequency;
    m_damping_ratio = props.damping_ratio;
    m_min_length = props.min_length;
    m_max_length = props.max_length;
    m_non_linear_terms = props.non_linear_terms;
    m_non_linear_contribution = props.non_linear_contribution;
}

float spring_joint2D::frequency() const
{
    return m_frequency;
}
void spring_joint2D::frequency(const float frequency)
{
    KIT_ASSERT_ERROR(frequency >= 0.f, "Frequency must be non-negative: {0}", frequency)
    m_frequency = frequency;
    awake();
}

float spring_joint2D::damping_ratio() const
{
    return m_damping_ratio;
}
void spring_joint2D::damping_ratio(const float damping_ratio)
{
    KIT_ASSERT_ERROR(damping_ratio >= 0.f, "Damping ratio must be non-negative: {0}", damping_ratio)
    m_damping_ratio = damping_ratio;
    awake();
}

float spring_joint2D::min_length() const
{
    return m_min_length;
}
void spring_joint2D::min_length(const float min_length)
{
    KIT_ASSERT_ERROR(min_length <= m_max_length, "Minimum length must be less than or equal to maximum length")
    m_min_length = min_length;
    awake();
}

float spring_joint2D::max_length() const
{
    return m_max_length;
}
void spring_joint2D::max_length(const float max_length)
{
    KIT_ASSERT_ERROR(m_min_length <= max_length, "Minimum length must be less than or equal to maximum length")
    m_max_length = max_length;
    awake();
}

std::uint32_t spring_joint2D::non_linear_terms() const
{
    return m_non_linear_terms;
}
void spring_joint2D::non_linear_terms(std::uint32_t non_linear_terms)
{
    m_non_linear_terms = non_linear_terms;
    awake();
}

float spring_joint2D::non_linear_contribution() const
{
    return m_non_linear_contribution;
}
void spring_joint2D::non_linear_contribution(const float non_linear_contribution)
{
    KIT_ASSERT_ERROR(non_linear_contribution >= 0.f, "Non linear contribution must be non-negative: {0}",
                     non_linear_contribution)
    m_non_linear_contribution = non_linear_contribution;
    awake();
}

std::pair<float, float> spring_joint2D::frequency_and_ratio(const float stiffness, const float damping,
                                                            const float mass)
{
    KIT_ASSERT_ERROR(stiffness >= 0.f, "Stiffness must be non-negative: {0}", stiffness)
    KIT_ASSERT_ERROR(damping >= 0.f, "Damping must be non-negative: {0}", damping)
    KIT_ASSERT_ERROR(mass > 0.f, "Mass must be positive: {0}", mass)

    const float frequency = glm::sqrt(stiffness / mass) / (2.f * glm::pi<float>());
    const float damping_ratio = damping / (4.f * glm::pi<float>() * mass * frequency);
    return {frequency, damping_ratio};
}

std::pair<float, float> spring_joint2D::stiffness_and_damping(const float frequency, const float damping_ratio,
                                                              const float mass)
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