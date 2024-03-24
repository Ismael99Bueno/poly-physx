#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/algorithm/intersection.hpp"

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

    for (auto it = m_collisions.begin(); it != m_collisions.end();)
    {
        if (it->collided)
            *it = generate_collision(*it->collider1, *it->collider2);
        if (!it->collided)
            it = m_collisions.erase(it);
        else
            ++it;
    }

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
    multithreaded = coldet.multithreaded;
    m_cp_narrow = std::move(coldet.m_cp_narrow);
    m_pp_narrow = std::move(coldet.m_pp_narrow);

    m_cc_manifold = std::move(coldet.m_cc_manifold);
    m_cp_manifold = std::move(coldet.m_cp_manifold);
    m_pp_manifold = std::move(coldet.m_pp_manifold);
}

void collision_detection2D::process_collision_st(collider2D &collider1, collider2D &collider2)
{
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
        m_collisions.push_back(colis);
}
void collision_detection2D::process_collision_mt(collider2D &collider1, collider2D &collider2,
                                                 const std::size_t thread_idx)
{
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
        m_mt_collisions[thread_idx].push_back(colis);
}
void collision_detection2D::join_mt_collisions()
{
    for (const auto &pairs : m_mt_collisions)
        m_collisions.insert(m_collisions.end(), pairs.begin(), pairs.end());
}

static bool broad_collision_check(const collider2D &collider1, const collider2D &collider2)
{
    const body2D &p1 = collider1.parent();
    const body2D &p2 = collider2.parent();
    return collider1 != collider2 && p1 != p2 &&
           (collider1.collision_filter.cgroup & collider2.collision_filter.collides_with) &&
           (collider2.collision_filter.cgroup & collider1.collision_filter.collides_with) &&
           (p1.is_dynamic() || p2.is_dynamic()) && geo::may_intersect(collider1.shape(), collider2.shape());
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

static void fill_collision_data(collision2D &collision, collider2D &collider1, collider2D &collider2,
                                const glm::vec2 &mtv, const manifold2D &manifold)
{
    collision.collided = true;
    collision.collider1 = &collider1;
    collision.collider2 = &collider2;
    collision.friction = sqrtf(collider1.friction * collider2.friction);
    collision.restitution = sqrtf(collider1.restitution * collider2.restitution);
    collision.mtv = mtv;
    collision.manifold = manifold;
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
    fill_collision_data(collision, collider1, collider2, mres.mtv,
                        m_cc_manifold->circle_circle_contacts(circ1, circ2, mres.mtv));
}
void collision_detection2D::cp_narrow_collision_check(collider2D &collider1, collider2D &collider2,
                                                      collision2D &collision) const
{
    const circle &circ = collider1.shape<circle>();
    const polygon &poly = collider2.shape<polygon>();

    const narrow_result nres = m_cp_narrow->circle_polygon(circ, poly);
    if (!nres.valid)
        return;

    fill_collision_data(collision, collider1, collider2, nres.mtv,
                        m_cp_manifold->circle_polygon_contacts(circ, poly, nres.mtv));
}
void collision_detection2D::pp_narrow_collision_check(collider2D &collider1, collider2D &collider2,
                                                      collision2D &collision) const
{
    const polygon &poly1 = collider1.shape<polygon>();
    const polygon &poly2 = collider2.shape<polygon>();

    const narrow_result nres = m_pp_narrow->polygon_polygon(poly1, poly2);
    if (!nres.valid)
        return;
    fill_collision_data(collision, collider1, collider2, nres.mtv,
                        m_pp_manifold->polygon_polygon_contacts(poly1, poly2, nres.mtv));
}

} // namespace ppx