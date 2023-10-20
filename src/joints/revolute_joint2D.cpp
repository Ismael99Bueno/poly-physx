#include "ppx/internal/pch.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
revolute_joint2D::revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2)
    : constraint2D("Revolute"), joint2D(body1, body2), m_length(glm::distance(body1->position(), body2->position()))
{
}

revolute_joint2D::revolute_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2)
    : constraint2D("Revolute"), joint2D(body1, body2, anchor1, anchor2),
      m_length(glm::distance(body1->position() + anchor1, body2->position() + anchor2))
{
}
revolute_joint2D::revolute_joint2D(const specs &spc)
    : constraint2D("Revolute"), joint2D(spc),
      m_length(glm::distance(spc.body1->position() + spc.anchor1, spc.body2->position() + spc.anchor2))
{
}

float revolute_joint2D::constraint_value() const
{
    const glm::vec2 p1 = rotated_anchor1() + m_body1->position(), p2 = rotated_anchor2() + m_body2->position();
    return glm::distance(p1, p2) - m_length;
}
float revolute_joint2D::constraint_derivative() const
{
    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();
    return glm::dot(dir, m_body1->velocity_at(rot_anchor1) - m_body2->velocity_at(rot_anchor2));
}

std::tuple<glm::vec2, glm::vec2, glm::vec2> revolute_joint2D::compute_anchors_and_direction() const
{
    const glm::vec2 rot_anchor1 = rotated_anchor1();
    const glm::vec2 rot_anchor2 = rotated_anchor2();
    const glm::vec2 dir = glm::normalize(rot_anchor1 - rot_anchor2 + m_body1->position() - m_body2->position());
    return {rot_anchor1, rot_anchor2, dir};
}

std::pair<float, float> revolute_joint2D::compute_impulses() const
{
    const float stiffness = 10.f;
    const float dampening = 1.f;
    const float c = constraint_value();

    const float bias = c * stiffness / (dampening + m_world->current_timestep() * stiffness);
    const float cdot = constraint_derivative() + bias;

    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();

    const float dot1 = glm::dot(rot_anchor1, dir);
    const float dot2 = glm::dot(rot_anchor2, dir);
    const float angular = glm::length2(rot_anchor1) - dot1 * dot1 + glm::length2(rot_anchor2) - dot2 * dot2;

    const float imp1 =
        -cdot / (2.f * m_body1->effective_inverse_mass() + angular * m_body1->effective_inverse_inertia());
    const float imp2 =
        -cdot / (2.f * m_body2->effective_inverse_mass() + angular * m_body2->effective_inverse_inertia());
    return {imp1, imp2};
}

// CHANGE TO ONLY ONE IMP FLOAT
void revolute_joint2D::apply_impulses(const float imp1, const float imp2)
{
    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();

    if (m_body1->kinematic)
    {
        m_body1->boost(m_body1->effective_inverse_mass() * imp1 * dir);
        m_body1->spin(m_body1->effective_inverse_inertia() * kit::cross2D(rot_anchor1, imp1 * dir));
    }
    if (m_body2->kinematic)
    {
        m_body2->boost(-m_body2->effective_inverse_mass() * imp2 * dir);
        m_body2->spin(m_body2->effective_inverse_inertia() * kit::cross2D(rot_anchor2, -imp2 * dir));
    }
}

void revolute_joint2D::warmup()
{
    if (kit::approaches_zero(m_accumulated_impulse1) && kit::approaches_zero(m_accumulated_impulse2))
        return;
    apply_impulses(m_accumulated_impulse1, m_accumulated_impulse2);
}
void revolute_joint2D::solve()
{
    const auto [imp1, imp2] = compute_impulses();
    m_accumulated_impulse1 += imp1;
    m_accumulated_impulse2 += imp2;

    apply_impulses(imp1, imp2);
}

bool revolute_joint2D::valid() const
{
    return joint2D::valid();
}
bool revolute_joint2D::contains(const kit::uuid id) const
{
    return m_body1->id == id || m_body2->id == id;
}
float revolute_joint2D::length() const
{
    return m_length;
}

revolute_joint2D::specs revolute_joint2D::specs::from_revolute_joint(const revolute_joint2D &rj)
{
    return {rj.body1(), rj.body2(), rj.rotated_anchor1(), rj.rotated_anchor2()};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node revolute_joint2D::encode() const
{
    const YAML::Node node1 = joint2D::encode();
    const YAML::Node node2 = constraint2D::encode();

    YAML::Node node;
    node["Joint2D"] = node1;
    node["Constraint2D"] = node2;

    return node;
}
bool revolute_joint2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() != 2)
        return false;

    if (!joint2D::decode(node["Joint2D"]))
        return false;
    if (!constraint2D::decode(node["Constraint2D"]))
        return false;
    return true;
}
#endif
} // namespace ppx