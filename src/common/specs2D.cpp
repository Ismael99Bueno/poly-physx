#include "ppx/internal/pch.hpp"
#include "ppx/common/specs2D.hpp"
#include "ppx/body/body2D.hpp"
#include "ppx/collider/collider2D.hpp"
#include "ppx/joints/spring_joint2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include "ppx/joints/weld_joint2D.hpp"
#include "ppx/joints/rotor_joint2D.hpp"
#include "ppx/joints/motor_joint2D.hpp"
#include "ppx/joints/ball_joint2D.hpp"
#include "ppx/joints/prismatic_joint2D.hpp"

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
            body.velocity(),
            body.rotation(),
            body.angular_velocity(),
            {body.props().nondynamic.mass, body.charge(), colliders, body.type()}};
}

rotor_joint2D rotor_joint2D::from_instance(const ppx::rotor_joint2D &rotj)
{
    rotor_joint2D specs{{rotj.body1()->index, rotj.body2()->index}, rotj.props()};
    specs.bodies_collide = rotj.bodies_collide;
    return specs;
}

motor_joint2D motor_joint2D::from_instance(const ppx::motor_joint2D &motj)
{
    motor_joint2D specs{{motj.body1()->index, motj.body2()->index}, motj.props()};
    specs.bodies_collide = motj.bodies_collide;
    return specs;
}

distance_joint2D distance_joint2D::from_instance(const ppx::distance_joint2D &dj)
{
    distance_joint2D specs{{dj.body1()->index, dj.body2()->index}, dj.ganchor1(), dj.ganchor2(), false, dj.props()};
    specs.bodies_collide = dj.bodies_collide;
    return specs;
}

revolute_joint2D revolute_joint2D::from_instance(const ppx::revolute_joint2D &revj)
{
    revolute_joint2D specs{{revj.body1()->index, revj.body2()->index}, revj.ganchor1()};
    specs.bodies_collide = revj.bodies_collide;
    return specs;
}

weld_joint2D weld_joint2D::from_instance(const ppx::weld_joint2D &weldj)
{
    weld_joint2D specs{{weldj.body1()->index, weldj.body2()->index}, weldj.ganchor1()};
    specs.bodies_collide = weldj.bodies_collide;
    return specs;
}

ball_joint2D ball_joint2D::from_instance(const ppx::ball_joint2D &bj)
{
    ball_joint2D specs{{bj.body1()->index, bj.body2()->index}, false, bj.props()};
    specs.bodies_collide = bj.bodies_collide;
    return specs;
}

prismatic_joint2D prismatic_joint2D::from_instance(const ppx::prismatic_joint2D &pj)
{
    prismatic_joint2D specs{{pj.body1()->index, pj.body2()->index}, pj.ganchor1(), pj.ganchor2(), false, pj.props()};
    specs.bodies_collide = pj.bodies_collide;
    return specs;
}

spring_joint2D spring_joint2D::from_instance(const ppx::spring_joint2D &sp)
{
    spring_joint2D specs{{sp.body1()->index, sp.body2()->index}, sp.ganchor1(), sp.ganchor2(), false, sp.props()};
    specs.bodies_collide = sp.bodies_collide;
    return specs;
}

void contraption2D::add_offset_to_joint_indices(const std::size_t offset)
{
    for (auto &dj : distance_joints)
    {
        dj.bindex1 += offset;
        dj.bindex2 += offset;
    }
    for (auto &sp : springs)
    {
        sp.bindex1 += offset;
        sp.bindex2 += offset;
    }
    for (auto &rj : revolute_joints)
    {
        rj.bindex1 += offset;
        rj.bindex2 += offset;
    }
    for (auto &wj : weld_joints)
    {
        wj.bindex1 += offset;
        wj.bindex2 += offset;
    }
    for (auto &rj : rotor_joints)
    {
        rj.bindex1 += offset;
        rj.bindex2 += offset;
    }
    for (auto &mj : motor_joints)
    {
        mj.bindex1 += offset;
        mj.bindex2 += offset;
    }
    for (auto &bj : ball_joints)
    {
        bj.bindex1 += offset;
        bj.bindex2 += offset;
    }
    for (auto &pj : prismatic_joints)
    {
        pj.bindex1 += offset;
        pj.bindex2 += offset;
    }
}

contraption2D contraption2D::rope(const glm::vec2 &start, const glm::vec2 &end, const std::uint32_t segments,
                                  const float anchor_spacing, const body2D::properties &body_props,
                                  const spring_joint2D::properties &spring_props, const bool fixed_start,
                                  const bool fixed_end)
{
    contraption2D contraption;
    contraption.springs.reserve(segments + 1);
    contraption.bodies.reserve(segments + 2);

    const glm::vec2 dir = (end - start) / (float)(segments + 1);
    const glm::vec2 spacing = 0.5f * anchor_spacing * glm::normalize(dir);
    const float rotation = atan2f(dir.y, dir.x);

    for (std::size_t i = 0; i < segments + 2; i++)
    {
        const glm::vec2 curpos = start + dir * (float)i;
        body2D specs;
        specs.position = curpos;
        specs.rotation = rotation;
        specs.props = body_props;
        contraption.bodies.push_back(specs);
    }
    if (fixed_start)
        contraption.bodies.front().props.type = body2D::btype::STATIC;
    if (fixed_end)
        contraption.bodies.back().props.type = body2D::btype::STATIC;

    glm::vec2 curpos = start;
    for (std::size_t i = 0; i < segments + 1; i++)
    {
        glm::vec2 nextpos = start + dir * (float)(i + 1);
        spring_joint2D specs;
        specs.bindex1 = i;
        specs.bindex2 = i + 1;
        specs.ganchor1 = curpos + spacing;
        specs.ganchor2 = nextpos - spacing;
        specs.deduce_length = false;
        specs.props = spring_props;
        contraption.springs.push_back(specs);
        curpos = nextpos;
    }
    return contraption;
}

contraption2D contraption2D::chain(const glm::vec2 &start, const glm::vec2 &end, const std::uint32_t segments,
                                   const float anchor_spacing, const body2D::properties &body_props,
                                   const distance_joint2D::properties &dj_props, const bool fixed_start,
                                   const bool fixed_end)
{
    contraption2D contraption;
    contraption.springs.reserve(segments + 1);
    contraption.bodies.reserve(segments + 2);

    const glm::vec2 dir = (end - start) / (float)(segments + 1);
    const glm::vec2 spacing = 0.5f * anchor_spacing * glm::normalize(dir);
    const float rotation = atan2f(dir.y, dir.x);

    for (std::size_t i = 0; i < segments + 2; i++)
    {
        const glm::vec2 curpos = start + dir * (float)i;
        body2D specs;
        specs.position = curpos;
        specs.rotation = rotation;
        specs.props = body_props;
        contraption.bodies.push_back(specs);
    }
    if (fixed_start)
        contraption.bodies.front().props.type = body2D::btype::STATIC;
    if (fixed_end)
        contraption.bodies.back().props.type = body2D::btype::STATIC;

    glm::vec2 curpos = start;
    for (std::size_t i = 0; i < segments + 1; i++)
    {
        glm::vec2 nextpos = start + dir * (float)(i + 1);
        distance_joint2D specs;
        specs.bindex1 = i;
        specs.bindex2 = i + 1;
        specs.ganchor1 = curpos + spacing;
        specs.ganchor2 = nextpos - spacing;
        specs.props = dj_props;
        specs.deduce_distance = true;
        contraption.distance_joints.push_back(specs);
        curpos = nextpos;
    }
    return contraption;
}

contraption2D contraption2D::soft_body(const std::vector<glm::vec2> &anchors, const body2D::properties &body_props,
                                       const spring_joint2D::properties &spring_props)
{
    return soft_body(anchors.begin(), anchors.end(), body_props, spring_props);
}
contraption2D contraption2D::soft_body(const float radius, const std::uint32_t segments,
                                       const body2D::properties &body_props,
                                       const spring_joint2D::properties &spring_props)
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