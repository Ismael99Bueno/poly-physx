#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/intersection.hpp"

#include "kit/utility/multithreading.hpp"

#if defined(PPX_MULTITHREADED) && defined(KIT_PROFILE)
#pragma message(                                                                                                       \
        "Multithreading for PPX will be disabled because the thread unsafe profiling features of cpp-kit are enabled")
#undef PPX_MULTITHREADED
#endif

namespace ppx
{

static bool are_both_circles(body2D &body1, body2D &body2)
{
    return body1.type() == body2D::shape_type::CIRCLE && body2.type() == body2D::shape_type::CIRCLE;
}

static bool broad_collision_check(body2D &body1, body2D &body2)
{
    return body1 != body2 && (body1.kinematic || body2.kinematic) && geo::may_intersect(body1.shape(), body2.shape());
}

const std::vector<collision2D> &collision_detection2D::detect_collisions_cached()
{
    KIT_PERF_FUNCTION()
    if (m_collisions.empty())
    {
        detect_collisions();
        m_collision_count = m_collisions.size();
        return m_collisions;
    }

#ifdef PPX_MULTITHREADED
    kit::for_each_mt<PPX_THREAD_COUNT, collision2D>(m_collisions, [this](std::size_t thread_index, collision2D &colis) {
        if (!narrow_collision_check(*colis.current, *colis.incoming, &colis))
            colis.valid = false;
    });
#else
    for (collision2D &colis : m_collisions)
        if (!narrow_collision_check(*colis.current, *colis.incoming, &colis))
            colis.valid = false;
#endif
    return m_collisions;
}
void collision_detection2D::clear_cached_collisions()
{
    m_collisions.clear();
#ifdef PPX_MULTITHREADED
    for (auto &collisions : m_mt_collisions)
        collisions.clear();
#endif
}

std::size_t collision_detection2D::last_collision_count() const
{
    return m_collision_count;
}

bool collision_detection2D::narrow_collision_check(body2D &body1, body2D &body2, collision2D *colis) const
{
    if (are_both_circles(body1, body2))
        return circle_narrow_collision_check(body1, body2, colis);
    return mixed_narrow_collision_check(body1, body2, colis);
}

bool collision_detection2D::gather_collision_data(body2D &body1, body2D &body2, collision2D *colis) const
{
    if (broad_collision_check(body1, body2))
        return narrow_collision_check(body1, body2, colis);
    return false;
}

bool collision_detection2D::circle_narrow_collision_check(body2D &body1, body2D &body2, collision2D *colis) const
{
    const geo::circle &c1 = body1.shape<geo::circle>(), &c2 = body2.shape<geo::circle>();
    if (!geo::intersects(c1, c2))
        return false;
    const glm::vec2 mtv = geo::mtv(c1, c2);
    const auto &[contact1, contact2] = geo::contact_points(c1, c2);
    *colis = {body1.as_ptr(), body2.as_ptr(), contact1, contact2, mtv};
    return true;
}

bool collision_detection2D::mixed_narrow_collision_check(body2D &body1, body2D &body2, collision2D *colis) const
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
    *colis = {body1.as_ptr(), body2.as_ptr(), contact1, contact2, mtv.value()};

    return true;
}

void collision_detection2D::try_enter_or_stay_callback(const collision2D &c) const
{
    c.current->events.try_enter_or_stay(c);
    c.incoming->events.try_enter_or_stay({c.incoming, c.current, c.touch2, c.touch1, -c.normal});
}
void collision_detection2D::try_exit_callback(body2D &body1, body2D &body2) const
{
    body1.events.try_exit(body1.as_ptr(), body2.as_ptr());
    body2.events.try_exit(body2.as_ptr(), body1.as_ptr());
}

} // namespace ppx