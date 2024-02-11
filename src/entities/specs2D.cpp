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
        const kit::transform2D<float> &transform = poly->ltransform();
        return {transform.position,   transform.rotation, collider.density(),   collider.charge_density(),
                collider.restitution, collider.friction,  poly->vertices.model, 0.f,
                collider.shape_type()};
    }

    const circle &circ = collider.shape<circle>();
    const kit::transform2D<float> &transform = circ.ltransform();
    return {transform.position,
            transform.rotation,
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

spring2D spring2D::from_spring(const ppx::spring2D &sp)
{
    return {{sp.joint.body1().raw(), sp.joint.body2().raw(), sp.joint.rotated_anchor1(), sp.joint.rotated_anchor2()},
            sp.stiffness,
            sp.damping,
            sp.length};
}

distance_joint2D distance_joint2D::from_distance_joint(const ppx::distance_joint2D &dj)
{
    return {{dj.joint.body1().raw(), dj.joint.body2().raw(), dj.joint.rotated_anchor1(), dj.joint.rotated_anchor2()}};
}
} // namespace ppx::specs