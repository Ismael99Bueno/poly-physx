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
float distance_joint2D::constraint_velocity() const
{
    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();
    return glm::dot(dir, joint.body1()->velocity_at(rot_anchor1) - joint.body2()->velocity_at(rot_anchor2));
}
float distance_joint2D::constraint_acceleration() const
{
    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();
    const body2D::ptr &body1 = joint.body1();
    const body2D::ptr &body2 = joint.body2();

    const glm::vec2 orth_anchor1 = glm::vec2(-rot_anchor1.y, rot_anchor1.x);
    const glm::vec2 orth_anchor2 = glm::vec2(-rot_anchor2.y, rot_anchor2.x);

    const glm::vec2 lin_term1 = body1->inv_mass() * body1->force() +
                                body1->inv_inertia() * body1->torque() * orth_anchor1 -
                                rot_anchor1 * body1->angular_velocity * body1->angular_velocity;
    const glm::vec2 lin_term2 = body2->inv_mass() * body2->force() +
                                body2->inv_inertia() * body2->torque() * orth_anchor2 -
                                rot_anchor2 * body2->angular_velocity * body2->angular_velocity;
    const float cross = kit::cross2D(dir, body1->velocity - body2->velocity);
    return glm::dot(dir, lin_term1 - lin_term2) +
           cross * cross / glm::distance(body1->position() + rot_anchor1, body2->position() + rot_anchor2);
}

std::tuple<glm::vec2, glm::vec2, glm::vec2> distance_joint2D::compute_anchors_and_direction() const
{
    const glm::vec2 rot_anchor1 = joint.rotated_anchor1();
    const glm::vec2 rot_anchor2 = joint.rotated_anchor2();
    const glm::vec2 dir =
        glm::normalize(rot_anchor1 - rot_anchor2 + joint.body1()->position() - joint.body2()->position());
    return {dir, rot_anchor1, rot_anchor2};
}

float distance_joint2D::compute_lambda() const
{
    const float c = constraint_value();
    const float cvel = constraint_velocity();

    static constexpr float stiffness = 100.f;
    static constexpr float dampening = 5.f;

    const float cacc = constraint_acceleration() + c * stiffness + cvel * dampening;

    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();

    const float cross1 = kit::cross2D(rot_anchor1, dir);
    const float cross2 = kit::cross2D(rot_anchor2, dir);

    const body2D::ptr &body1 = joint.body1();
    const body2D::ptr &body2 = joint.body2();

    const float inv_mass = body1->inv_mass() + body2->inv_mass() + body1->inv_inertia() * cross1 * cross1 +
                           body2->inv_inertia() * cross2 * cross2;
    return -cacc / inv_mass;
}

void distance_joint2D::apply_lambda(const float lambda)
{
    const auto [dir, rot_anchor1, rot_anchor2] = compute_anchors_and_direction();
    const glm::vec2 f1 = lambda * dir;
    const glm::vec2 f2 = -f1;

    const body2D::ptr &body1 = joint.body1();
    const body2D::ptr &body2 = joint.body2();

    body1->apply_simulation_force_at(f1, rot_anchor1);
    body2->apply_simulation_force_at(f2, rot_anchor2);
}

void distance_joint2D::warmup()
{
    if (kit::approaches_zero(m_accumulated_lambda))
        return;
    m_accumulated_lambda *= world->timestep_ratio();
    apply_lambda(m_accumulated_lambda);
}
void distance_joint2D::solve()
{
    const float lambda = compute_lambda();
    m_accumulated_lambda += lambda;
    apply_lambda(lambda);
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

    if (!joint.decode(node["Joint2D"], *world))
        return false;
    if (!constraint2D::decode(node["Constraint2D"]))
        return false;
    length = node["Length"].as<float>();
    return true;
}
#endif
} // namespace ppx