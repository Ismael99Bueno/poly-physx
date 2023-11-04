#include "ppx/internal/pch.hpp"
#include "ppx/joints/spring2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
spring2D::spring2D(const float stiffness, const float dampening, const float length)
    : stiffness(stiffness), dampening(dampening), length(length)
{
}
spring2D::spring2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                   const glm::vec2 &anchor2, const float stiffness, const float dampening, const float length)
    : joint(body1, body2, anchor1, anchor2), stiffness(stiffness), dampening(dampening), length(length)
{
}
spring2D::spring2D(const specs &spc)
    : joint(spc.joint), stiffness(spc.stiffness), dampening(spc.dampening), length(spc.length)
{
}

glm::vec4 spring2D::force() const
{
    const body2D::ptr &body1 = joint.body1();
    const body2D::ptr &body2 = joint.body2();

    const glm::vec2 rot_anchor1 = joint.rotated_anchor1(), rot_anchor2 = joint.rotated_anchor2();
    const glm::vec2 p1 = body1->position() + rot_anchor1, p2 = body2->position() + rot_anchor2;
    const glm::vec2 relpos = p2 - p1, direction = glm::normalize(relpos),
                    relvel = direction *
                             glm::dot(body2->velocity_at(rot_anchor2) - body1->velocity_at(rot_anchor1), direction),
                    vlen = length * direction;

    const glm::vec2 force = stiffness * (relpos - vlen) + dampening * relvel;
    const float torque1 = kit::cross2D(rot_anchor1, force), torque2 = kit::cross2D(force, rot_anchor2);
    return {force, torque1, torque2};
}

float spring2D::kinetic_energy() const
{
    return joint.body1()->kinetic_energy() + joint.body2()->kinetic_energy();
}
float spring2D::potential_energy() const
{
    const glm::vec2 p1 = joint.body1()->position() + joint.rotated_anchor1(),
                    p2 = joint.body2()->position() + joint.rotated_anchor2();
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

spring2D::specs spring2D::specs::from_spring(const spring2D &sp)
{
    return {{sp.joint.body1(), sp.joint.body2(), sp.joint.rotated_anchor1(), sp.joint.rotated_anchor2()},
            sp.stiffness,
            sp.dampening,
            sp.length};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node spring2D::serializer::encode(const spring2D &sp) const
{
    YAML::Node node = sp.joint.encode();
    node["UUID"] = (std::uint64_t)sp.id;
    node["Stiffness"] = sp.stiffness;
    node["Dampening"] = sp.dampening;
    node["Length"] = sp.length;
    return node;
}
bool spring2D::serializer::decode(const YAML::Node &node, spring2D &sp) const
{
    if (!sp.joint.decode(node, *sp.m_world))
        return false;
    sp.id = kit::uuid(node["UUID"].as<std::uint64_t>());
    sp.stiffness = node["Stiffness"].as<float>();
    sp.dampening = node["Dampening"].as<float>();
    sp.length = node["Length"].as<float>();
    return true;
}
#endif
} // namespace ppx