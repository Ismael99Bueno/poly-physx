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
        return {collider.lposition(), collider.lrotation(), collider.density(),   collider.charge_density(),
                collider.restitution, collider.friction,    poly->vertices.model, 0.f,
                collider.shape_type()};
    }

    const circle &circ = collider.shape<circle>();
    return {collider.lposition(),
            collider.lrotation(),
            collider.density(),
            collider.charge_density(),
            collider.restitution,
            collider.friction,
            {},
            circ.radius(),
            collider.shape_type()};
}

body2D body2D::from_body(const ppx::body2D &body)
{
    std::vector<collider2D> colliders;
    colliders.reserve(body.size());

    for (const ppx::collider2D &collider : body)
        colliders.push_back(collider2D::from_collider(collider));

    return {body.position(), body.velocity, body.rotation(), body.angular_velocity, body.props().nondynamic.mass,
            body.charge,     colliders,     body.type()};
}

joint_proxy2D joint_proxy2D::from_joint_proxy(const ppx::joint_proxy2D &jp)
{
    return {jp.body1()->index, jp.body2()->index, jp.rotated_anchor1(), jp.rotated_anchor2()};
}

spring2D spring2D::from_spring(const ppx::spring2D &sp)
{
    return {joint_proxy2D::from_joint_proxy(sp.joint), sp.stiffness, sp.damping, sp.length};
}

distance_joint2D distance_joint2D::from_distance_joint(const ppx::distance_joint2D &dj)
{
    return {joint_proxy2D::from_joint_proxy(dj.joint)};
}
} // namespace ppx::specs