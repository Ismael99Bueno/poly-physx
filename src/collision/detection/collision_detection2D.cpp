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
        return m_collisions;
    }

#ifdef PPX_MULTITHREADED
    kit::for_each_mt<PPX_THREAD_COUNT, collision2D>(m_collisions, [this](std::size_t thread_index, collision2D &colis) {
        if (!narrow_collision_check(*colis.body1, *colis.body2, &colis))
            colis.valid = false;
    });
#else
    for (collision2D &colis : m_collisions)
        if (!narrow_collision_check(*colis.body1, *colis.body2, &colis))
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

const std::vector<collision2D> &collision_detection2D::collisions() const
{
    return m_collisions;
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
    const geo::mtv_result mres = geo::mtv(c1, c2);
    if (!mres.valid)
        return false;

    *colis = {body1.as_ptr(), body2.as_ptr(), mres.mtv, {geo::contact_point(c1, c2)}};
    return true;
}

bool collision_detection2D::mixed_narrow_collision_check(body2D &body1, body2D &body2, collision2D *colis) const
{
    const geo::shape2D &sh1 = body1.shape(), &sh2 = body2.shape();
    if (!geo::may_intersect(sh1, sh2))
        return false;

    const geo::gjk_result gres = geo::gjk(sh1, sh2);
    if (!gres.intersect)
        return false;

    const geo::mtv_result mres = geo::epa(sh1, sh2, gres.simplex);
    if (!mres.valid)
        return false;

    *colis = {body1.as_ptr(), body2.as_ptr(), mres.mtv, {geo::contact_point(sh1, sh2, mres.mtv)}};

    return true;
}

void collision_detection2D::try_enter_or_stay_callback(const collision2D &c) const
{
    c.body1->events.try_enter_or_stay(c);
    c.body2->events.try_enter_or_stay(c);
}
void collision_detection2D::try_exit_callback(body2D &body1, body2D &body2) const
{
    body1.events.try_exit(body1.as_ptr(), body2.as_ptr());
    body2.events.try_exit(body2.as_ptr(), body1.as_ptr());
}

} // namespace ppx