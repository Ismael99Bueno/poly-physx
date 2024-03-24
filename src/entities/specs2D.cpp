#include "ppx/internal/pch.hpp"
#include "ppx/entities/specs2D.hpp"
#include "ppx/entities/body2D.hpp"
#include "ppx/entities/collider2D.hpp"
#include "ppx/joints/spring2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"

namespace ppx::specs
{
collider2D collider2D::from_instance(const ppx::collider2D &collider)
{
    if (const auto *poly = collider.shape_if<polygon>())
    {
        return {collider.lposition(),
                collider.lrotation(),
                {collider.density(), collider.charge_density(), collider.restitution, collider.friction,
                 poly->vertices.model, 0.f, collider.shape_type(), collider.collision_filter}};
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

body2D body2D::from_instance(const ppx::body2D &body)
{
    std::vector<collider2D> colliders;
    colliders.reserve(body.size());

    for (const ppx::collider2D *collider : body)
        colliders.push_back(collider2D::from_instance(*collider));

    return {body.gposition(),
            body.velocity,
            body.rotation(),
            body.angular_velocity,
            {body.props().nondynamic.mass, body.charge, colliders, body.type()}};
}

distance_joint2D distance_joint2D::from_instance(const ppx::distance_joint2D &dj)
{
    return {{dj.body1()->index, dj.body2()->index}, dj.ganchor1(), dj.ganchor2()};
}

spring2D spring2D::from_instance(const ppx::spring2D &sp)
{
    return {{sp.body1()->index, sp.body2()->index},
            sp.ganchor1(),
            sp.ganchor2(),
            {sp.stiffness, sp.damping, sp.length, sp.non_linear_terms, sp.non_linear_contribution}};
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
        const glm::vec2 curpos = start + dir * (float)i;
        const glm::vec2 &prevpos = contraption.bodies.back().position;
        contraption.bodies.push_back(body2D{.position = start + dir * (float)i, .props = node_props});
        contraption.springs.push_back({{i - 1, i}, prevpos + spacing, curpos - spacing, spring_props});
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
        contraption.distance_joints.push_back({{i - 1, i}, spacing, -spacing});
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