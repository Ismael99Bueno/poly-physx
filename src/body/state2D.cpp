#include "ppx/internal/pch.hpp"
#include "ppx/body/state2D.hpp"

namespace ppx
{
glm::vec2 state2D::local_centroid_point(const glm::vec2 &gpoint) const
{
    return centroid.inverse_center_scale_rotate_translate3() * glm::vec3(gpoint, 1.f);
}
glm::vec2 state2D::global_centroid_point(const glm::vec2 &lpoint) const
{
    return centroid.center_scale_rotate_translate3() * glm::vec3(lpoint, 1.f);
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
    return centroid.inverse_center_scale_rotate_translate3() * glm::vec3(gvector, 0.f);
}
glm::vec2 state2D::global_vector(const glm::vec2 &lvector) const
{
    return centroid.center_scale_rotate_translate3() * glm::vec3(lvector, 0.f);
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
} // namespace ppx