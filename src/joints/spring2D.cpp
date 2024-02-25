#include "ppx/internal/pch.hpp"
#include "ppx/joints/spring2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
spring2D::spring2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                   const glm::vec2 &anchor2, const float stiffness, const float damping, const float length,
                   const std::uint32_t non_linear_terms, const float non_linear_contribution)
    : kit::identifiable<>(kit::uuid::random()), worldref2D(world), joint(body1, body2, anchor1, anchor2),
      stiffness(stiffness), damping(damping), length(length), non_linear_terms(non_linear_terms)
{
}
spring2D::spring2D(world2D &world, const specs &spc)
    : kit::identifiable<>(kit::uuid::random()), worldref2D(world), joint(world.bodies, spc.joint),
      stiffness(spc.props.stiffness), damping(spc.props.damping), length(spc.props.length),
      non_linear_terms(spc.props.non_linear_terms), non_linear_contribution(spc.props.non_linear_contribution)
{
}

spring2D::const_ptr spring2D::as_ptr() const
{
    return world.springs.ptr(index);
}
spring2D::ptr spring2D::as_ptr()
{
    return world.springs.ptr(index);
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
    KIT_ASSERT_ERROR(stiffness >= 0.f, "Stiffness must be non-negative: {0}", stiffness)
    KIT_ASSERT_ERROR(damping >= 0.f, "Damping must be non-negative: {0}", damping)
    KIT_ASSERT_ERROR(length >= 0.f, "Length must be non-negative: {0}", length)
    KIT_ASSERT_ERROR(non_linear_contribution >= 0.f, "Non linear contribution must be non-negative: {0}",
                     non_linear_contribution)

    const body2D::ptr &body1 = joint.body1();
    const body2D::ptr &body2 = joint.body2();

    const glm::vec2 rot_anchor1 = joint.rotated_anchor1(), rot_anchor2 = joint.rotated_anchor2();
    const glm::vec2 p1 = body1->centroid() + rot_anchor1, p2 = body2->centroid() + rot_anchor2;

    const glm::vec2 relpos = p2 - p1;
    const glm::vec2 direction = glm::normalize(relpos);
    const glm::vec2 relvel =
        direction * glm::dot(body2->velocity_at(rot_anchor2) - body1->velocity_at(rot_anchor1), direction);
    const glm::vec2 vlen = length * direction;

    const glm::vec2 displacement = relpos - vlen;
    const glm::vec2 force =
        stiffness * (non_linear_terms != 0 ? non_linear_displacement(displacement) : displacement) + damping * relvel;

    const float torque1 = kit::cross2D(rot_anchor1, force), torque2 = kit::cross2D(force, rot_anchor2);
    return {force, torque1, torque2};
}

float spring2D::kinetic_energy() const
{
    return joint.body1()->kinetic_energy() + joint.body2()->kinetic_energy();
}
float spring2D::potential_energy() const
{
    const glm::vec2 p1 = joint.body1()->centroid() + joint.rotated_anchor1(),
                    p2 = joint.body2()->centroid() + joint.rotated_anchor2();
    const float dist = glm::distance(p1, p2) - length;
    return 0.5f * stiffness * dist * dist;
}
float spring2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

void spring2D::apply_force_to_bodies()
{
    const body2D::ptr &body1 = joint.body1();
    const body2D::ptr &body2 = joint.body2();

    const glm::vec4 f = force();
    body1->apply_simulation_force(glm::vec2(f));
    body2->apply_simulation_force(-glm::vec2(f));

    body1->apply_simulation_torque(f.z);
    body2->apply_simulation_torque(f.w);
}
} // namespace ppx