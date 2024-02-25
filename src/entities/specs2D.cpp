#include "ppx/internal/pch.hpp"
#include "ppx/entities/specs2D.hpp"
#include "ppx/entities/body2D.hpp"
#include "ppx/entities/collider2D.hpp"
#include "ppx/joints/spring2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"

namespace ppx::specs
{
collider2D collider2D::from_collider(const ppx::collider2D &collider)
{
    if (const auto *poly = collider.shape_if<polygon>())
    {
        return {collider.lposition(),
                collider.lrotation(),
                {collider.density(), collider.charge_density(), collider.restitution, collider.friction,
                 poly->vertices.model, 0.f, collider.shape_type()}};
    }

    const circle &circ = collider.shape<circle>();
    return {collider.lposition(),
            collider.lrotation(),
            {collider.density(),
             collider.charge_density(),
             collider.restitution,
             collider.friction,
             {},
             circ.radius(),
             collider.shape_type()}};
}

body2D body2D::from_body(const ppx::body2D &body)
{
    std::vector<collider2D> colliders;
    colliders.reserve(body.size());

    for (const ppx::collider2D &collider : body)
        colliders.push_back(collider2D::from_collider(collider));

    return {body.position(),
            body.velocity,
            body.rotation(),
            body.angular_velocity,
            {body.props().nondynamic.mass, body.charge, colliders, body.type()}};
}

joint_proxy2D joint_proxy2D::from_joint_proxy(const ppx::joint_proxy2D &jp)
{
    return {jp.body1()->index, jp.body2()->index, jp.rotated_anchor1(), jp.rotated_anchor2()};
}

spring2D spring2D::from_spring(const ppx::spring2D &sp)
{
    return {joint_proxy2D::from_joint_proxy(sp.joint), {sp.stiffness, sp.damping, sp.length}};
}

distance_joint2D distance_joint2D::from_distance_joint(const ppx::distance_joint2D &dj)
{
    return {joint_proxy2D::from_joint_proxy(dj.joint)};
}

contraption2D contraption2D::rope(const glm::vec2 &start, const glm::vec2 &end, const std::uint32_t segments,
                                  const float spring_anchor_spacing, const body2D::properties &node_props,
                                  const spring2D::properties &spring_props, const bool fixed_start,
                                  const bool fixed_end)
{
    contraption2D contraption;
    contraption.bodies.reserve(segments + 2);
    contraption.springs.reserve(segments + 1);

    const glm::vec2 dir = (end - start) / (float)(segments + 1);
    const glm::vec2 spacing = 0.5f * spring_anchor_spacing * glm::normalize(dir);

    contraption.bodies.push_back(body2D{.position = start, .props = node_props});
    for (std::size_t i = 1; i <= segments + 1; i++)
    {
        contraption.bodies.push_back(body2D{.position = start + dir * (float)i, .props = node_props});
        contraption.springs.push_back(
            spring2D{.joint = joint_proxy2D{.bindex1 = i - 1, .bindex2 = i, .anchor1 = spacing, .anchor2 = -spacing},
                     .props = spring_props});
    }
    return contraption;
}

contraption2D contraption2D::chain(const glm::vec2 &start, const glm::vec2 &end, const std::uint32_t segments,
                                   const float spring_anchor_spacing, const body2D::properties &node_props,
                                   const bool fixed_start, const bool fixed_end)
{
    contraption2D contraption;
    contraption.bodies.reserve(segments + 2);
    contraption.distance_joints.reserve(segments + 1);

    const glm::vec2 dir = (end - start) / (float)(segments + 1);
    const glm::vec2 spacing = 0.5f * spring_anchor_spacing * glm::normalize(dir);

    contraption.bodies.push_back(body2D{.position = start, .props = node_props});
    for (std::size_t i = 1; i <= segments + 1; i++)
    {
        contraption.bodies.push_back(body2D{.position = start + dir * (float)i, .props = node_props});
        contraption.distance_joints.push_back(distance_joint2D{
            .joint = joint_proxy2D{.bindex1 = i - 1, .bindex2 = i, .anchor1 = spacing, .anchor2 = -spacing}});
    }
    return contraption;
}

contraption2D contraption2D::soft_body(const std::vector<glm::vec2> &anchors, const body2D::properties &body_props,
                                       const spring2D::properties &spring_props)
{
    return soft_body(anchors.begin(), anchors.end(), body_props, spring_props);
}
contraption2D contraption2D::soft_body(const float radius, const std::uint32_t segments,
                                       const body2D::properties &body_props, const spring2D::properties &spring_props)
{
    std::vector<glm::vec2> anchors;
    anchors.reserve(segments);
    for (std::uint32_t i = 0; i < segments; i++)
    {
        const float angle = 2.f * glm::pi<float>() * (float)i / (float)segments;
        anchors.push_back(radius * glm::vec2{glm::cos(angle), glm::sin(angle)});
    }
    return soft_body(anchors.begin(), anchors.end(), body_props, spring_props);
}

} // namespace ppx::specs