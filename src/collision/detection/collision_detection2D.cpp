#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
collision_detection2D::collision_detection2D(world2D &world) : worldref2D(world)
{
}
const collision_detection2D::collision_map &collision_detection2D::detect_collisions_cached()
{
    KIT_PERF_FUNCTION()
#ifdef KIT_PROFILE
    KIT_ASSERT_ERROR(!multithreaded, "Cannot run multiple threads if the KIT profiling tools are enabled")
#endif
    if (m_new_step)
    {
        detect_collisions();
        m_new_step = false;
        return m_collisions;
    }

    for (auto it = m_collisions.begin(); it != m_collisions.end();)
    {
        it->second = generate_collision(it->second.collider1, it->second.collider2);
        if (!it->second.collided)
            it = m_collisions.erase(it);
        else
            ++it;
    }
    return m_collisions;
}

const collision_detection2D::collision_map &collision_detection2D::collisions() const
{
    return m_collisions;
}

void collision_detection2D::flag_new_step()
{
    m_new_step = true;
    m_last_collisions.swap(m_collisions);
    m_collisions.clear();
    for (auto &pairs : m_mt_collisions)
        pairs.clear();
}

void collision_detection2D::inherit(collision_detection2D &&coldet)
{
    multithreaded = coldet.multithreaded;
    m_cp_narrow = std::move(coldet.m_cp_narrow);
    m_pp_narrow = std::move(coldet.m_pp_narrow);

    m_pp_manifold = std::move(coldet.m_pp_manifold);

    m_collisions = std::move(coldet.m_collisions);
    m_last_collisions = std::move(coldet.m_last_collisions);
}

void collision_detection2D::process_collision_st(collider2D *collider1, collider2D *collider2)
{
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
    {
        kit::commutative_tuple<const collider2D *, const collider2D *> hash{collider1, collider2};
        m_collisions.emplace(hash, colis);
        KIT_ASSERT_ERROR(colis.friction >= 0.f, "Friction must be non-negative: {0}", colis.friction)
        KIT_ASSERT_ERROR(colis.restitution >= 0.f, "Restitution must be non-negative: {0}", colis.restitution)
    }
}
void collision_detection2D::process_collision_mt(collider2D *collider1, collider2D *collider2,
                                                 const std::size_t thread_idx)
{
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
    {
        kit::commutative_tuple<const collider2D *, const collider2D *> hash{collider1, collider2};
        m_mt_collisions[thread_idx].emplace(hash, colis);
        KIT_ASSERT_ERROR(colis.friction >= 0.f, "Friction must be non-negative: {0}", colis.friction)
        KIT_ASSERT_ERROR(colis.restitution >= 0.f, "Restitution must be non-negative: {0}", colis.restitution)
    }
}
void collision_detection2D::join_mt_collisions()
{
    for (const auto &pairs : m_mt_collisions)
        m_collisions.insert(pairs.begin(), pairs.end());
}

static bool elligible_for_collision(const collider2D *collider1, const collider2D *collider2)
{
    const body2D *body1 = collider1->body();
    const body2D *body2 = collider2->body();
    return collider1 != collider2 && body1 != body2 &&
           (collider1->collision_filter.cgroups & collider2->collision_filter.collides_with) &&
           (collider2->collision_filter.cgroups & collider1->collision_filter.collides_with) &&
           geo::may_intersect(collider1->shape(), collider2->shape()) && !body1->joint_prevents_collision(body2);
}

collision2D collision_detection2D::generate_collision(collider2D *collider1, collider2D *collider2) const
{
    collision2D collision;
    const body2D *body1 = collider1->body();
    const body2D *body2 = collider2->body();
    if (!body1->is_dynamic() && !body2->is_dynamic())
        return collision;

    if (collider1->body()->asleep() && collider2->body()->asleep())
    {
        const auto last_collision = m_last_collisions.find({collider1, collider2});
        if (last_collision != m_last_collisions.end())
        {
            collision = last_collision->second;
            collision.asleep = true;
            return collision;
        }
        return collision;
    }
    if (!elligible_for_collision(collider1, collider2))
        return collision;
    if (collider1->is_circle() && collider2->is_circle())
        cc_narrow_collision_check(collider1, collider2, collision);
    else if (collider1->is_polygon() && collider2->is_polygon())
        pp_narrow_collision_check(collider1, collider2, collision);
    else if (collider1->is_polygon() && collider2->is_circle())
        cp_narrow_collision_check(collider2, collider1, collision);
    else if (collider1->is_circle() && collider2->is_polygon())
        cp_narrow_collision_check(collider1, collider2, collision);
    return collision;
}

static void fill_collision_data(collision2D &collision, collider2D *collider1, collider2D *collider2,
                                const glm::vec2 &mtv)
{
    collision.collided = true;
    collision.collider1 = collider1;
    collision.collider2 = collider2;
    collision.friction = sqrtf(collider1->friction * collider2->friction);
    collision.restitution = sqrtf(collider1->restitution * collider2->restitution);
    collision.mtv = mtv;
}

void collision_detection2D::cc_narrow_collision_check(collider2D *collider1, collider2D *collider2,
                                                      collision2D &collision) const
{
    const circle &circ1 = collider1->shape<circle>();
    const circle &circ2 = collider2->shape<circle>();

    if (!geo::intersects(circ1, circ2))
        return;
    const geo::mtv_result2D mres = geo::mtv(circ1, circ2);
    if (!mres)
        return;

    fill_collision_data(collision, collider1, collider2, mres.mtv);
    collision.manifold = {geo::radius_distance_contact_point(circ1, circ2, mres.mtv)};
}
void collision_detection2D::cp_narrow_collision_check(collider2D *collider1, collider2D *collider2,
                                                      collision2D &collision) const
{
    const circle &circ = collider1->shape<circle>();
    const polygon &poly = collider2->shape<polygon>();

    const narrow_result nres = m_cp_narrow->circle_polygon(circ, poly);
    if (!nres)
        return;

    fill_collision_data(collision, collider1, collider2, nres.mtv);
    collision.manifold = {geo::radius_penetration_contact_point(circ, nres.mtv)};
}
void collision_detection2D::pp_narrow_collision_check(collider2D *collider1, collider2D *collider2,
                                                      collision2D &collision) const
{
    const polygon &poly1 = collider1->shape<polygon>();
    const polygon &poly2 = collider2->shape<polygon>();

    const narrow_result nres = m_pp_narrow->polygon_polygon(poly1, poly2);
    if (!nres)
        return;

    fill_collision_data(collision, collider1, collider2, nres.mtv);

    const auto pcontact = m_last_collisions.find({collider1, collider2});
    collision.manifold = m_pp_manifold->polygon_polygon_contacts(
        collision, pcontact != m_last_collisions.end() ? &pcontact->second : nullptr);
    collision.collided = !collision.manifold.empty();
}

} // namespace ppx