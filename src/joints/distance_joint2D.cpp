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
    return {dir, rot_anchor1, rot_anchor2};
}

float distance_joint2D::compute_impulse() const
{
    const float cdot = constraint_derivative();

    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();

    const float cross1 = kit::cross2D(rot_anchor1, dir);
    const float cross2 = kit::cross2D(rot_anchor2, dir);

    const float inv_mass = joint.body1()->effective_inverse_mass() + joint.body2()->effective_inverse_mass() +
                           joint.body1()->effective_inverse_inertia() * cross1 * cross1 +
                           joint.body2()->effective_inverse_inertia() * cross2 * cross2;
    return -cdot / inv_mass;
}

void distance_joint2D::apply_impulse(const float imp)
{
    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();
    const glm::vec2 imp1 = imp * dir;
    const glm::vec2 imp2 = -imp1;

    if (joint.body1()->kinematic) // REMOVE THIS
    {
        joint.body1()->boost(joint.body1()->effective_inverse_mass() * imp1);
        joint.body1()->spin(joint.body1()->effective_inverse_inertia() * kit::cross2D(rot_anchor1, imp1));
    }
    if (joint.body2()->kinematic)
    {
        joint.body2()->boost(joint.body2()->effective_inverse_mass() * imp2);
        joint.body2()->spin(joint.body2()->effective_inverse_inertia() * kit::cross2D(rot_anchor2, imp2));
    }
}

void distance_joint2D::warmup()
{
    return;
    if (kit::approaches_zero(m_accumulated_impulse))
        return;
    m_accumulated_impulse *= m_world->timestep_ratio();
    apply_impulse(m_accumulated_impulse);
}
void distance_joint2D::solve()
{
    const float imp = compute_impulse();
    m_accumulated_impulse += imp;
    apply_impulse(imp);
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