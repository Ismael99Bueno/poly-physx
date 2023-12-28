#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/algorithm/intersection.hpp"

#include "kit/utility/multithreading.hpp"

#if defined(PPX_MULTITHREADED) && defined(KIT_PROFILE)
#pragma message(                                                                                                       \
        "Multithreading for PPX will be disabled because the thread unsafe profiling features of cpp-kit are enabled")
#undef PPX_MULTITHREADED
#endif

namespace ppx
{
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
        if (colis.collided)
            colis = generate_collision(*colis.body1, *colis.body2);
    });
#else
    for (collision2D &colis : m_collisions)
        if (colis.collided)
            colis = generate_collision(*colis.body1, *colis.body2);
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

void collision_detection2D::inherit(collision_detection2D &coldet)
{
    epa_threshold = coldet.epa_threshold;
    m_cc_manifold = std::move(coldet.m_cc_manifold);
    m_cp_manifold = std::move(coldet.m_cp_manifold);
    m_pp_manifold = std::move(coldet.m_pp_manifold);
}

static bool broad_collision_check(body2D &body1, body2D &body2)
{
    return body1 != body2 && (body1.kinematic || body2.kinematic) && geo::may_intersect(body1.shape(), body2.shape());
}

collision2D collision_detection2D::generate_collision(body2D &body1, body2D &body2) const
{
    collision2D collision;
    if (!broad_collision_check(body1, body2))
        return collision;
    if (body1.is_circle() && body2.is_circle())
        cc_narrow_collision_check(body1, body2, collision);
    else if (body1.is_polygon() && body2.is_polygon())
        pp_narrow_collision_check(body1, body2, collision);
    else if (body1.is_polygon() && body2.is_circle())
        cp_narrow_collision_check(body2, body1, collision);
    else if (body1.is_circle() && body2.is_polygon())
        cp_narrow_collision_check(body1, body2, collision);
    return collision;
}

void collision_detection2D::cc_narrow_collision_check(body2D &body1, body2D &body2, collision2D &collision) const
{
    const geo::circle &circ1 = body1.shape<geo::circle>();
    const geo::circle &circ2 = body2.shape<geo::circle>();

    if (!geo::intersects(circ1, circ2))
        return;
    const geo::mtv_result mres = geo::mtv(circ1, circ2);
    if (!mres.valid)
        return;
    collision.collided = true;
    collision.body1 = body1.as_ptr();
    collision.body2 = body2.as_ptr();
    collision.mtv = mres.mtv;
    collision.manifold = m_cc_manifold->circle_circle_contacts(circ1, circ2, mres.mtv);
}
void collision_detection2D::cp_narrow_collision_check(body2D &body1, body2D &body2, collision2D &collision) const
{
    const geo::circle &circ = body1.shape<geo::circle>();
    const geo::polygon &poly = body2.shape<geo::polygon>();

    const geo::gjk_result gres = geo::gjk(circ, poly);
    if (!gres.intersect)
        return;

    const geo::mtv_result mres = geo::epa(circ, poly, gres.simplex, epa_threshold);
    if (!mres.valid)
        return;

    collision.collided = true;
    collision.body1 = body1.as_ptr();
    collision.body2 = body2.as_ptr();
    collision.mtv = mres.mtv;
    collision.manifold = m_cp_manifold->circle_polygon_contacts(circ, poly, mres.mtv);
}
void collision_detection2D::pp_narrow_collision_check(body2D &body1, body2D &body2, collision2D &collision) const
{
    const geo::polygon &poly1 = body1.shape<geo::polygon>();
    const geo::polygon &poly2 = body2.shape<geo::polygon>();

    const geo::gjk_result gres = geo::gjk(poly1, poly2);
    if (!gres.intersect)
        return;

    const geo::mtv_result mres = geo::epa(poly1, poly2, gres.simplex, epa_threshold);
    if (!mres.valid)
        return;

    collision.collided = true;
    collision.body1 = body1.as_ptr();
    collision.body2 = body2.as_ptr();
    collision.mtv = mres.mtv;
    collision.manifold = m_pp_manifold->polygon_polygon_contacts(poly1, poly2, mres.mtv);
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