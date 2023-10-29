#include "ppx/internal/pch.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
distance_joint2D::distance_joint2D() : constraint2D("Distance")
{
}
distance_joint2D::distance_joint2D(const body2D::ptr &body1, const body2D::ptr &body2, const glm::vec2 &anchor1,
                                   const glm::vec2 &anchor2)
    : constraint2D("Distance"), length(glm::distance(body1->position() + anchor1, body2->position() + anchor2))
{
}
distance_joint2D::distance_joint2D(const specs &spc)
    : constraint2D("Distance"), joint(spc.joint), length(glm::distance(spc.joint.body1->position() + spc.joint.anchor1,
                                                                       spc.joint.body2->position() + spc.joint.anchor2))
{
}

float distance_joint2D::constraint_value() const
{
    const glm::vec2 p1 = joint.rotated_anchor1() + joint.body1()->position(),
                    p2 = joint.rotated_anchor2() + joint.body2()->position();
    return glm::distance(p1, p2) - length;
}
float distance_joint2D::constraint_derivative() const
{
    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();
    return glm::dot(dir, joint.body1()->velocity_at(rot_anchor1) - joint.body2()->velocity_at(rot_anchor2));
}

std::tuple<glm::vec2, glm::vec2, glm::vec2> distance_joint2D::compute_anchors_and_direction() const
{
    const glm::vec2 rot_anchor1 = joint.rotated_anchor1();
    const glm::vec2 rot_anchor2 = joint.rotated_anchor2();
    const glm::vec2 dir =
        glm::normalize(rot_anchor1 - rot_anchor2 + joint.body1()->position() - joint.body2()->position());
    return {rot_anchor1, rot_anchor2, dir};
}

std::pair<float, float> distance_joint2D::compute_impulses() const
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
        -cdot / (2.f * joint.body1()->effective_inverse_mass() + angular * joint.body1()->effective_inverse_inertia());
    const float imp2 =
        -cdot / (2.f * joint.body2()->effective_inverse_mass() + angular * joint.body2()->effective_inverse_inertia());
    return {imp1, imp2};
}

// CHANGE TO ONLY ONE IMP FLOAT
void distance_joint2D::apply_impulses(const float imp1, const float imp2)
{
    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();

    if (joint.body1()->kinematic)
    {
        joint.body1()->boost(joint.body1()->effective_inverse_mass() * imp1 * dir);
        joint.body1()->spin(joint.body1()->effective_inverse_inertia() * kit::cross2D(rot_anchor1, imp1 * dir));
    }
    if (joint.body2()->kinematic)
    {
        joint.body2()->boost(-joint.body2()->effective_inverse_mass() * imp2 * dir);
        joint.body2()->spin(joint.body2()->effective_inverse_inertia() * kit::cross2D(rot_anchor2, -imp2 * dir));
    }
}

void distance_joint2D::warmup()
{
    if (kit::approaches_zero(m_accumulated_impulse1) && kit::approaches_zero(m_accumulated_impulse2))
        return;
    apply_impulses(m_accumulated_impulse1, m_accumulated_impulse2);
}
void distance_joint2D::solve()
{
    const auto [imp1, imp2] = compute_impulses();
    m_accumulated_impulse1 += imp1;
    m_accumulated_impulse2 += imp2;

    apply_impulses(imp1, imp2);
}

bool distance_joint2D::valid() const
{
    return joint.valid();
}
bool distance_joint2D::contains(const kit::uuid id) const
{
    return joint.body1()->id == id || joint.body2()->id == id;
}

distance_joint2D::specs distance_joint2D::specs::from_distance_joint(const distance_joint2D &dj)
{
    return {{dj.joint.body1(), dj.joint.body2(), dj.joint.rotated_anchor1(), dj.joint.rotated_anchor2()}};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node distance_joint2D::encode() const
{
    const YAML::Node node1 = joint.encode();
    const YAML::Node node2 = constraint2D::encode();

    YAML::Node node;
    node["Joint2D"] = node1;
    node["Constraint2D"] = node2;
    node["Length"] = length;

    return node;
}
bool distance_joint2D::decode(const YAML::Node &node)
{
    if (!node.IsMap() || node.size() != 3)
        return false;

    if (!joint.decode(node["Joint2D"], *m_world))
        return false;
    if (!constraint2D::decode(node["Constraint2D"]))
        return false;
    length = node["Length"].as<float>();
    return true;
}
#endif
} // namespace ppx