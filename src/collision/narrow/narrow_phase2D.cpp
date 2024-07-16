#include "ppx/internal/pch.hpp"
#include "ppx/collision/narrow/narrow_phase2D.hpp"
#include "kit/multithreading/mt_for_each.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
const std::vector<collision2D> &narrow_phase2D::compute_collisions(const std::vector<cpair> &pairs)
{
    KIT_PERF_SCOPE("narrow_phase")
    if (world.rk_subset_index() == 0)
    {
        flush_collisions();
        if (params.multithreading)
            compute_collisions_mt(pairs);
        else
            compute_collisions_st(pairs);
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

void narrow_phase2D::compute_collisions_st(const std::vector<cpair> &pairs)
{
    for (const cpair &pair : pairs)
        process_collision_st(pair.first, pair.second);
}
void narrow_phase2D::compute_collisions_mt(const std::vector<cpair> &pairs)
{
    const auto lambda = [this](const std::size_t workload_index, const cpair &pair) {
        process_collision_mt(pair.first, pair.second, workload_index);
    };
    kit::mt::for_each(*world.thread_pool, pairs, lambda, params.parallel_workloads);
    join_mt_collisions();
}

const std::vector<collision2D> &narrow_phase2D::collisions() const
{
    return m_collisions;
}

const char *narrow_phase2D::name() const
{
    return "Unnamed";
}

narrow_phase2D::result::operator bool() const
{
    return intersects;
}

void narrow_phase2D::flush_collisions()
{
    m_collisions.clear();
    m_mt_collisions.resize(params.parallel_workloads);
    for (std::size_t i = 0; i < params.parallel_workloads; i++)
        m_mt_collisions[i].clear();
}

void narrow_phase2D::process_collision_st(collider2D *collider1, collider2D *collider2)
{
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
    {
        m_collisions.push_back(colis);
        KIT_ASSERT_ERROR(colis.friction >= 0.f, "Friction must be non-negative: {0}", colis.friction)
        KIT_ASSERT_ERROR(colis.restitution >= 0.f, "Restitution must be non-negative: {0}", colis.restitution)
    }
}
void narrow_phase2D::process_collision_mt(collider2D *collider1, collider2D *collider2,
                                          const std::size_t workload_index)
{
    const collision2D colis = generate_collision(collider1, collider2);
    if (colis.collided)
    {
        m_mt_collisions[workload_index].push_back(colis);
        KIT_ASSERT_ERROR(colis.friction >= 0.f, "Friction must be non-negative: {0}", colis.friction)
        KIT_ASSERT_ERROR(colis.restitution >= 0.f, "Restitution must be non-negative: {0}", colis.restitution)
    }
}
void narrow_phase2D::join_mt_collisions()
{
    for (std::size_t i = 0; i < params.parallel_workloads; i++)
    {
        const auto &pairs = m_mt_collisions[i];
        m_collisions.insert(m_collisions.end(), pairs.begin(), pairs.end());
    }
}

bool narrow_phase2D::is_potential_collision(const collider2D *collider1,
                                            const collider2D *collider2) // this must be tuned
{
    const body2D *body1 = collider1->body();
    const body2D *body2 = collider2->body();
    return (body1->is_dynamic() || body2->is_dynamic()) && (!body1->asleep() || !body2->asleep()) &&
           (collider1->collision_filter.cgroups & collider2->collision_filter.collides_with) &&
           (collider2->collision_filter.cgroups & collider1->collision_filter.collides_with) &&
           geo::intersects(collider1->tight_bbox(), collider2->tight_bbox()) && !body1->joint_prevents_collision(body2);
}

collision2D narrow_phase2D::generate_collision(collider2D *collider1, collider2D *collider2) const
{
    collision2D collision;
    if (!is_potential_collision(collider1, collider2))
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
