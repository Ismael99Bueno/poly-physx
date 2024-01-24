#include "ppx/internal/pch.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/serialization/serialization.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
distance_joint2D::distance_joint2D(world2D &world) : joint_constraint2D(world, "Distance")
{
}
distance_joint2D::distance_joint2D(world2D &world, const body2D::ptr &body1, const body2D::ptr &body2,
                                   const glm::vec2 &anchor1, const glm::vec2 &anchor2)
    : joint_constraint2D(world, "Distance"),
      length(glm::distance(body1->position() + anchor1, body2->position() + anchor2))
{
}
distance_joint2D::distance_joint2D(world2D &world, const specs &spc)
    : joint_constraint2D(world, "Distance"), joint(spc.joint),
      length(glm::distance(spc.joint.body1->position() + spc.joint.anchor1,
                           spc.joint.body2->position() + spc.joint.anchor2))
{
}

float distance_joint2D::constraint_value() const
{
    KIT_ASSERT_ERROR(length >= 0.f, "Length must be non-negative: {0}", length)
    const glm::vec2 p1 = joint.rotated_anchor1() + joint.body1()->position(),
                    p2 = joint.rotated_anchor2() + joint.body2()->position();
    return length - glm::distance(p1, p2);
}
float distance_joint2D::constraint_velocity() const
{
    const auto [dir, rot_anchor1, rot_anchor2] = joint.compute_anchors_and_direction();
    return glm::dot(dir, joint.body2()->constraint_velocity_at(rot_anchor2) -
                             joint.body1()->constraint_velocity_at(rot_anchor1));
}

void distance_joint2D::warmup()
{
    const auto [dir, rot_anchor1, rot_anchor2] = joint.compute_anchors_and_direction();
    apply_warmup(*joint.body1(), *joint.body2(), rot_anchor1, rot_anchor2, dir);
}
void distance_joint2D::solve()
{
    const auto [dir, rot_anchor1, rot_anchor2] = joint.compute_anchors_and_direction();
    solve_unclamped(*joint.body1(), *joint.body2(), rot_anchor1, rot_anchor2, dir);
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
    return kit::yaml::codec<distance_joint2D>::encode(*this);
}
bool distance_joint2D::decode(const YAML::Node &node)
{
    return kit::yaml::codec<distance_joint2D>::decode(node, *this);
}
#endif
} // namespace ppx