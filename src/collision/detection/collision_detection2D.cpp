#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/algorithm/intersection.hpp"
#include "kit/multithreading/mt_for_each.hpp"

namespace ppx
{
collision_detection2D::collision_detection2D(world2D &world) : worldref2D(world)
{
}
const std::vector<collision2D> &collision_detection2D::detect_collisions_cached()
{
    KIT_PERF_FUNCTION()
#ifdef KIT_PROFILE
    KIT_ASSERT_ERROR(!multithreaded, "Cannot run multiple threads if the KIT profiling tools are enabled")
#endif
    if (m_collisions.empty())
    {
        detect_collisions();
        return m_collisions;
    }

    if (multithreaded)
        kit::mt::for_each<PPX_THREAD_COUNT>(m_collisions, [this](std::size_t thread_index, collision2D &colis) {
            if (colis.collided)
                colis = generate_collision(*colis.collider1, *colis.collider2);
        });
    else
        for (collision2D &colis : m_collisions)
            if (colis.collided)
                colis = generate_collision(*colis.collider1, *colis.collider2);

    return m_collisions;
}
void collision_detection2D::clear_cached_collisions()
{
    m_collisions.clear();
    if (multithreaded)
        for (auto &collisions : m_mt_collisions)
            collisions.clear();
}

const std::vector<collision2D> &collision_detection2D::collisions() const
{
    return m_collisions;
}

void collision_detection2D::inherit(collision_detection2D &coldet)
{
    epa_threshold = coldet.epa_threshold;
    multithreaded = coldet.multithreaded;
    m_cc_manifold = std::move(coldet.m_cc_manifold);
    m_cp_manifold = std::move(coldet.m_cp_manifold);
    m_pp_manifold = std::move(coldet.m_pp_manifold);
}

void collision_detection2D::process_collision_st(collider2D &collider1, collider2D &collider2)
{
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
    {
        try_enter_or_stay_callback(colis);
        m_collisions.push_back(colis);
    }
    else
        try_exit_callback(collider1, collider2);
}
void collision_detection2D::process_collision_mt(collider2D &collider1, collider2D &collider2,
                                                 const std::size_t thread_idx)
{
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
    {
        try_enter_or_stay_callback(colis);
        m_mt_collisions[thread_idx].push_back(colis);
    }
    else
        try_exit_callback(collider1, collider2);
}
void collision_detection2D::join_mt_collisions()
{
    for (const auto &pairs : m_mt_collisions)
        m_collisions.insert(m_collisions.end(), pairs.begin(), pairs.end());
}

static bool broad_collision_check(collider2D &collider1, collider2D &collider2)
{
    const body2D &p1 = collider1.parent();
    const body2D &p2 = collider2.parent();
    return collider1 != collider2 && p1 != p2 && (p1.is_dynamic() || p2.is_dynamic()) &&
           geo::may_intersect(collider1.shape(), collider2.shape());
}

collision2D collision_detection2D::generate_collision(collider2D &collider1, collider2D &collider2) const
{
    collision2D collision;
    if (!broad_collision_check(collider1, collider2))
        return collision;
    if (collider1.is_circle() && collider2.is_circle())
        cc_narrow_collision_check(collider1, collider2, collision);
    else if (collider1.is_polygon() && collider2.is_polygon())
        pp_narrow_collision_check(collider1, collider2, collision);
    else if (collider1.is_polygon() && collider2.is_circle())
        cp_narrow_collision_check(collider2, collider1, collision);
    else if (collider1.is_circle() && collider2.is_polygon())
        cp_narrow_collision_check(collider1, collider2, collision);
    return collision;
}

void collision_detection2D::cc_narrow_collision_check(collider2D &collider1, collider2D &collider2,
                                                      collision2D &collision) const
{
    const circle &circ1 = collider1.shape<circle>();
    const circle &circ2 = collider2.shape<circle>();

    if (!geo::intersects(circ1, circ2))
        return;
    const geo::mtv_result mres = geo::mtv(circ1, circ2);
    if (!mres.valid)
        return;
    collision.collided = true;
    collision.collider1 = &collider1;
    collision.collider2 = &collider2;
    collision.friction = sqrtf(collider1.friction * collider2.friction);
    collision.restitution = sqrtf(collider1.restitution * collider2.restitution);
    collision.mtv = mres.mtv;
    collision.manifold = m_cc_manifold->circle_circle_contacts(circ1, circ2, mres.mtv);
}
void collision_detection2D::cp_narrow_collision_check(collider2D &collider1, collider2D &collider2,
                                                      collision2D &collision) const
{
    const circle &circ = collider1.shape<circle>();
    const polygon &poly = collider2.shape<polygon>();

    const geo::gjk_result gres = geo::gjk(circ, poly);
    if (!gres.intersect)
        return;

    const geo::mtv_result mres = geo::epa(circ, poly, gres.simplex, epa_threshold);
    if (!mres.valid)
        return;

    collision.collided = true;
    collision.collider1 = &collider1;
    collision.collider2 = &collider2;
    collision.friction = sqrtf(collider1.friction * collider2.friction);
    collision.restitution = sqrtf(collider1.restitution * collider2.restitution);
    collision.mtv = mres.mtv;
    collision.manifold = m_cp_manifold->circle_polygon_contacts(circ, poly, mres.mtv);
}
void collision_detection2D::pp_narrow_collision_check(collider2D &collider1, collider2D &collider2,
                                                      collision2D &collision) const
{
    const polygon &poly1 = collider1.shape<polygon>();
    const polygon &poly2 = collider2.shape<polygon>();

    const geo::gjk_result gres = geo::gjk(poly1, poly2);
    if (!gres.intersect)
        return;

    const geo::mtv_result mres = geo::epa(poly1, poly2, gres.simplex, epa_threshold);
    if (!mres.valid)
        return;

    collision.collided = true;
    collision.collider1 = &collider1;
    collision.collider2 = &collider2;
    collision.friction = sqrtf(collider1.friction * collider2.friction);
    collision.restitution = sqrtf(collider1.restitution * collider2.restitution);
    collision.mtv = mres.mtv;
    collision.manifold = m_pp_manifold->polygon_polygon_contacts(poly1, poly2, mres.mtv);
}

void collision_detection2D::try_enter_or_stay_callback(const collision2D &c) const
{
    c.collider1->events.try_enter_or_stay(c, *c.collider2);
    c.collider2->events.try_enter_or_stay(c, *c.collider1);
}
void collision_detection2D::try_exit_callback(collider2D &collider1, collider2D &collider2) const
{
    collider1.events.try_exit(collider1, collider2);
    collider2.events.try_exit(collider2, collider1);
}

} // namespace ppx