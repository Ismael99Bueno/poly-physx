#include "ppx/internal/pch.hpp"
#include "ppx/body/state2D.hpp"

namespace ppx
{
state2D::state2D(const transform2D &centroid, const glm::vec2 &velocity, const float angular_velocity)
    : centroid(centroid), lposition(0.f), velocity(velocity), angular_velocity(angular_velocity)
{
}

glm::vec2 state2D::local_centroid_point(const glm::vec2 &gpoint) const
{
    return inv_tmatrix() * glm::vec3(gpoint, 1.f);
}
glm::vec2 state2D::global_centroid_point(const glm::vec2 &lpoint) const
{
    return tmatrix() * glm::vec3(lpoint, 1.f);
}

glm::vec2 state2D::local_position_point(const glm::vec2 &gpoint) const
{
    return local_centroid_point(gpoint) - lposition;
}
glm::vec2 state2D::global_position_point(const glm::vec2 &lpoint) const
{
    return global_centroid_point(lpoint + lposition);
}

glm::vec2 state2D::local_vector(const glm::vec2 &gvector) const
{
    return inv_tmatrix() * glm::vec3(gvector, 0.f);
}
glm::vec2 state2D::global_vector(const glm::vec2 &lvector) const
{
    return tmatrix() * glm::vec3(lvector, 0.f);
}

glm::vec2 state2D::lvelocity_at_from_centroid(const glm::vec2 &lpoint) const
{
    return gvelocity_at(global_centroid_point(lpoint));
}
glm::vec2 state2D::lvelocity_at_from_position(const glm::vec2 &lpoint) const
{
    return gvelocity_at(global_position_point(lpoint));
}
glm::vec2 state2D::gvelocity_at(const glm::vec2 &gpoint) const
{
    return velocity_at_centroid_offset(gpoint - centroid.position);
}
glm::vec2 state2D::velocity_at_centroid_offset(const glm::vec2 &offset) const
{
    return velocity + angular_velocity * glm::vec2(-offset.y, offset.x);
}
glm::vec2 state2D::velocity_at_position_offset(const glm::vec2 &offset) const
{
    return velocity_at_centroid_offset(offset + lposition);
}

glm::mat3 state2D::tmatrix() const
{
    return centroid.center_scale_rotate_translate3();
}
glm::mat3 state2D::inv_tmatrix() const
{
    return centroid.inverse_center_scale_rotate_translate3();
}

float state2D::inv_mass() const
{
    return is_dynamic() ? imass : 0.f;
}
float state2D::inv_inertia() const
{
    return is_dynamic() ? iinertia : 0.f;
}

bool state2D::is_dynamic() const
{
    return type == specs::body2D::btype::DYNAMIC;
}
bool state2D::is_kinematic() const
{
    return type == specs::body2D::btype::KINEMATIC;
}
bool state2D::is_static() const
{
    return type == specs::body2D::btype::STATIC;
}

float state2D::kinetic_energy() const
{
    return 0.5f * (mass * glm::length2(velocity) + inertia * angular_velocity * angular_velocity);
}

} // namespace ppx