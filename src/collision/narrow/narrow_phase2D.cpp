#include "ppx/internal/pch.hpp"
#include "ppx/collision/narrow/narrow_phase2D.hpp"
#include "kit/multithreading/mt_for_each.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
const std::vector<collision2D> &narrow_phase2D::compute_collisions(const std::vector<pair> &new_pairs)
{
    KIT_PERF_SCOPE("narrow_phase")
    if (world.rk_subset_index() == 0)
    {
        for (const auto &np : new_pairs)
            if (m_unique_pairs.emplace(np.collider1, np.collider2).second)
                m_pairs.push_back(np);

        m_collisions.clear();
        if (params.multithreading && world.thread_pool)
            compute_collisions_mt();
        else
            compute_collisions_st();
        remove_outdated_pairs();
        return m_collisions;
    }

    for (auto it = m_collisions.begin(); it != m_collisions.end();)
    {
        *it = generate_collision(it->collider1, it->collider2);
        if (!it->collided)
            it = m_collisions.erase(it);
        else
            ++it;
    }
    return m_collisions;
}

void narrow_phase2D::compute_collisions_st()
{
    for (const pair &p : m_pairs)
        process_collision(p.collider1, p.collider2);
}
void narrow_phase2D::compute_collisions_mt()
{
    const auto lambda = [this](const std::size_t workload_index, const pair &p) {
        process_collision(p.collider1, p.collider2);
    };
    kit::mt::for_each(*world.thread_pool, m_pairs, lambda, params.parallel_workloads);
}

void narrow_phase2D::remove_pairs_containing(const collider2D *collider)
{
    for (std::size_t i = m_pairs.size() - 1; i != SIZE_MAX; --i)
    {
        const pair &p = m_pairs[i];
        if (p.collider1 == collider || p.collider2 == collider)
        {
            m_unique_pairs.erase({p.collider1, p.collider2});
            m_pairs.erase(m_pairs.begin() + i);
        }
    }
}

const std::vector<collision2D> &narrow_phase2D::collisions() const
{
    return m_collisions;
}
const std::vector<narrow_phase2D::pair> &narrow_phase2D::pairs() const
{
    return m_pairs;
}

const char *narrow_phase2D::name() const
{
    return "Unnamed";
}

narrow_phase2D::result::operator bool() const
{
    return intersects;
}

static bool is_potential_collision(const collider2D *collider1,
                                   const collider2D *collider2) // this must be tuned
{
    const body2D *body1 = collider1->body();
    const body2D *body2 = collider2->body();
    return (body1->is_dynamic() || body2->is_dynamic()) && (!body1->asleep() || !body2->asleep()) &&
           (collider1->collision_filter.cgroups & collider2->collision_filter.collides_with) &&
           (collider2->collision_filter.cgroups & collider1->collision_filter.collides_with) &&
           geo::intersects(collider1->tight_bbox(), collider2->tight_bbox()) && !body1->joint_prevents_collision(body2);
}

void narrow_phase2D::process_collision(collider2D *collider1, collider2D *collider2)
{
    if (!is_potential_collision(collider1, collider2))
        return;
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
    {
        KIT_ASSERT_ERROR(colis.friction >= 0.f, "Friction must be non-negative: {0}", colis.friction)
        KIT_ASSERT_ERROR(colis.restitution >= 0.f, "Restitution must be non-negative: {0}", colis.restitution)

        static std::mutex mutex;
        std::scoped_lock lock(mutex);
        m_collisions.push_back(colis);
    }
}

void narrow_phase2D::remove_outdated_pairs()
{
    for (std::size_t i = m_pairs.size() - 1; i != SIZE_MAX; --i)
    {
        const pair &p = m_pairs[i];
        if (!geo::intersects(p.collider1->fat_bbox(), p.collider2->fat_bbox()))
        {
            m_unique_pairs.erase({p.collider1, p.collider2});
            m_pairs.erase(m_pairs.begin() + i);
        }
    }
}

collision2D narrow_phase2D::generate_collision(collider2D *collider1, collider2D *collider2) const
{
    collision2D collision;
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
                                const glm::vec2 &mtv, const manifold2D &manifold)
{
    collision.collided = true;
    collision.collider1 = collider1;
    collision.collider2 = collider2;
    collision.friction = sqrtf(collider1->friction * collider2->friction);
    collision.restitution = sqrtf(collider1->restitution * collider2->restitution);
    collision.mtv = mtv;
    collision.manifold = manifold;
}

void narrow_phase2D::cc_narrow_collision_check(collider2D *collider1, collider2D *collider2,
                                               collision2D &collision) const
{
    const circle &circ1 = collider1->shape<circle>();
    const circle &circ2 = collider2->shape<circle>();

    if (!geo::intersects(circ1, circ2))
        return;
    const geo::mtv_result2D mres = geo::mtv(circ1, circ2);
    if (!mres)
        return;

    const manifold2D manifold = {geo::radius_distance_contact_point(circ1, circ2, mres.mtv)};
    fill_collision_data(collision, collider1, collider2, mres.mtv, manifold);
}
void narrow_phase2D::cp_narrow_collision_check(collider2D *collider1, collider2D *collider2,
                                               collision2D &collision) const
{
    const circle &circ = collider1->shape<circle>();
    const polygon &poly = collider2->shape<polygon>();
    const float R = circ.radius() + poly.radius();
    if (glm::distance2(circ.gcentroid(), poly.gcentroid()) > R * R)
        return;

    const result nres = circle_polygon(circ, poly);
    if (!nres)
        return;

    fill_collision_data(collision, collider1, collider2, nres.mtv, nres.manifold);
}
void narrow_phase2D::pp_narrow_collision_check(collider2D *collider1, collider2D *collider2,
                                               collision2D &collision) const
{
    const polygon &poly1 = collider1->shape<polygon>();
    const polygon &poly2 = collider2->shape<polygon>();
    const float R = poly1.radius() + poly2.radius();
    if (glm::distance2(poly1.gcentroid(), poly2.gcentroid()) > R * R)
        return;

    const result nres = polygon_polygon(poly1, poly2);
    if (!nres)
        return;

    fill_collision_data(collision, collider1, collider2, nres.mtv, nres.manifold);
}

} // namespace ppx
