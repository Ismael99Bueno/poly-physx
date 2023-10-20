#include "ppx/internal/pch.hpp"
#include "ppx/joints/spring2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
spring2D::spring2D(const body2D::ptr &body1, const body2D::ptr &body2, const float stiffness, const float dampening,
                   const float length)
    : joint2D(body1, body2), m_stiffness(stiffness), m_dampening(dampening), m_length(length)
{
}

spring2D::spring2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                   const glm::vec2 &anchor2, const float stiffness, const float dampening, const float length)
    : joint2D(body1, body2, anchor1, anchor2), m_stiffness(stiffness), m_dampening(dampening), m_length(length)
{
}
spring2D::spring2D(const specs &spc)
    : joint2D(spc), m_stiffness(spc.stiffness), m_dampening(spc.dampening), m_length(spc.length)
{
}

glm::vec4 spring2D::force() const
{
    const glm::vec2 rot_anchor1 = rotated_anchor1(), rot_anchor2 = rotated_anchor2();
    const glm::vec2 p1 = m_body1->position() + rot_anchor1, p2 = m_body2->position() + rot_anchor2;
    const glm::vec2 relpos = p2 - p1, direction = glm::normalize(relpos),
                    relvel = direction *
                             glm::dot(m_body2->velocity_at(rot_anchor2) - m_body1->velocity_at(rot_anchor1), direction),
                    vlen = m_length * direction;

    const glm::vec2 force = m_stiffness * (relpos - vlen) + m_dampening * relvel;
    const float torque1 = kit::cross2D(rot_anchor1, force), torque2 = kit::cross2D(force, rot_anchor2);
    return {force, torque1, torque2};
}

float spring2D::length() const
{
    return m_length;
}
void spring2D::length(const float length)
{
    m_length = length;
}

float spring2D::stiffness() const
{
    return m_stiffness;
}
float spring2D::dampening() const
{
    return m_dampening;
}

void spring2D::stiffness(const float stiffness)
{
    m_stiffness = stiffness;
}
void spring2D::dampening(const float dampening)
{
    m_dampening = dampening;
}

float spring2D::kinetic_energy() const
{
    return m_body1->kinetic_energy() + m_body2->kinetic_energy();
}
float spring2D::potential_energy() const
{
    const glm::vec2 p1 = m_body1->position() + rotated_anchor1(), p2 = m_body2->position() + rotated_anchor2();
    const float dist = glm::distance(p1, p2) - m_length;
    return 0.5f * m_stiffness * dist * dist;
}
float spring2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

void spring2D::apply_force_to_bodies()
{
    const glm::vec4 f = force();
    m_body1->apply_simulation_force(glm::vec2(f));
    m_body2->apply_simulation_force(-glm::vec2(f));

    m_body1->apply_simulation_torque(f.z);
    m_body2->apply_simulation_torque(f.w);
}

spring2D::specs spring2D::specs::from_spring(const spring2D &sp)
{
    return {{sp.body1(), sp.body2(), sp.rotated_anchor1(), sp.rotated_anchor2()},
            sp.stiffness(),
            sp.dampening(),
            sp.length()};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node spring2D::encode() const
{
    YAML::Node node = joint2D::encode();
    node["UUID"] = (std::uint64_t)id;
    node["Stiffness"] = m_stiffness;
    node["Dampening"] = m_dampening;
    node["Length"] = m_length;
    return node;
}
bool spring2D::decode(const YAML::Node &node)
{
    if (!joint2D::decode(node))
        return false;
    id = kit::uuid(node["UUID"].as<std::uint64_t>());
    m_stiffness = node["Stiffness"].as<float>();
    m_dampening = node["Dampening"].as<float>();
    m_length = node["Length"].as<float>();
    return true;
}
#endif
} // namespace ppx