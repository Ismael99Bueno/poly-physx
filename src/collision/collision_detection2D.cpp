#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision_detection2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/intersection.hpp"

namespace ppx
{
collision_detection2D::collision_detection2D(world2D &parent) : m_parent(parent)
{
}

static bool are_both_circles(const body2D &body1, const body2D &body2)
{
    return body1.type() == body2D::shape_type::CIRCLE && body2.type() == body2D::shape_type::CIRCLE;
}

static bool broad_collision_check(const body2D &body1, const body2D &body2)
{
    return body1 != body2 && (body1.kinematic || body2.kinematic) && geo::may_intersect(body1.shape(), body2.shape());
}

const std::vector<collision2D> &collision_detection2D::cached_collisions()
{
    return m_collisions.empty() ? detect_collisions() : m_collisions;
}
void collision_detection2D::flush_collisions()
{
    m_collisions.clear();
#ifdef PPX_MULTITHREADED
    for (auto &collisions : m_mt_collisions)
        collisions.clear();
#endif
}

bool collision_detection2D::narrow_collision_check(const body2D &body1, const body2D &body2, collision2D *colis) const
{
    if (are_both_circles(body1, body2))
        return circle_narrow_collision_check(body1, body2, colis);
    return mixed_narrow_collision_check(body1, body2, colis);
}

bool collision_detection2D::are_colliding(const body2D &body1, const body2D &body2, collision2D *colis) const
{
    if (broad_collision_check(body1, body2))
        return narrow_collision_check(body1, body2, colis);
    return false;
}

bool collision_detection2D::circle_narrow_collision_check(const body2D &body1, const body2D &body2,
                                                          collision2D *colis) const
{
    const geo::circle &c1 = body1.shape<geo::circle>(), &c2 = body2.shape<geo::circle>();
    if (!geo::intersect(c1, c2))
        return false;
    const glm::vec2 mtv = geo::mtv(c1, c2);
    const auto &[contact1, contact2] = geo::contact_points(c1, c2);
    *colis = {m_parent[body1.index], m_parent[body2.index], contact1, contact2, mtv};
    return true;
}

bool collision_detection2D::mixed_narrow_collision_check(const body2D &body1, const body2D &body2,
                                                         collision2D *colis) const
{
    const geo::shape2D &sh1 = body1.shape(), &sh2 = body2.shape();
    if (!geo::may_intersect(sh1, sh2))
        return false;

    auto simplex = geo::gjk(sh1, sh2);
    if (!simplex)
        return false;

    auto mtv = geo::epa(sh1, sh2, simplex.value());
    if (!mtv)
        return false;

    const auto &[contact1, contact2] = geo::contact_points(sh1, sh2, mtv.value());
    *colis = {m_parent[body1.index], m_parent[body2.index], contact1, contact2, mtv.value()};

    return true;
}

void collision_detection2D::try_enter_or_stay_callback(const collision2D &c) const
{
    c.current->events.try_enter_or_stay(c);
    c.incoming->events.try_enter_or_stay({c.incoming, c.current, c.touch2, c.touch1, -c.normal});
}
void collision_detection2D::try_exit_callback(const body2D &body1, const body2D &body2) const
{
    body1.events.try_exit(m_parent[body1.index], m_parent[body2.index]);
    body2.events.try_exit(m_parent[body2.index], m_parent[body1.index]);
}

} // namespace ppx