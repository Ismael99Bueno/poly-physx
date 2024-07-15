#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
broad_phase2D::broad_phase2D(world2D &world) : worldref2D(world)
{
}

const char *broad_phase2D::name() const
{
    return "Unnamed";
}

const std::vector<collision2D> &broad_phase2D::detect_collisions_cached(const cp_narrow_phase2D *cp_narrow,
                                                                        const pp_narrow_phase2D *pp_narrow)
{
    KIT_PERF_SCOPE("broad_phase2D")
    m_cp_narrow = cp_narrow;
    m_pp_narrow = pp_narrow;
    if (world.rk_subset_index() == 0)
    {
        detect_collisions();
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

float broad_phase2D::metrics::accuracy() const
{
    return total_collision_checks > 0 ? (float)positive_collision_checks / (float)total_collision_checks : 0.f;
}

broad_phase2D::metrics broad_phase2D::collision_metrics() const
{
    return m_metrics;
}
std::vector<broad_phase2D::metrics> broad_phase2D::collision_metrics_per_mt_workload() const
{
    KIT_ASSERT_WARN(params.multithreading,
                    "Per-thread metrics are meaningless if the broad phase is not multithreading")
    return m_mt_metrics;
}

const std::vector<collision2D> &broad_phase2D::collisions() const
{
    return m_collisions;
}

void broad_phase2D::flush_collisions()
{
    m_collisions.clear();
    m_metrics = {};
    m_mt_collisions.resize(params.parallel_workloads);
    m_mt_metrics.resize(params.parallel_workloads);
    for (std::size_t i = 0; i < params.parallel_workloads; i++)
    {
        m_mt_collisions[i].clear();
        m_mt_metrics[i] = {};
    }
}

void broad_phase2D::inherit(broad_phase2D &&broad)
{
    params = broad.params;
    m_collisions = std::move(broad.m_collisions);
}

void broad_phase2D::process_collision_st(collider2D *collider1, collider2D *collider2)
{
    KIT_PERF_SCOPE("broad_phase2D::process_collision_st")
    const collision2D colis = generate_collision(collider1, collider2);
    m_metrics.total_collision_checks++;
    if (colis.collided)
    {
        m_collisions.push_back(colis);
        m_metrics.positive_collision_checks++;
        KIT_ASSERT_ERROR(colis.friction >= 0.f, "Friction must be non-negative: {0}", colis.friction)
        KIT_ASSERT_ERROR(colis.restitution >= 0.f, "Restitution must be non-negative: {0}", colis.restitution)
    }
}
void broad_phase2D::process_collision_mt(collider2D *collider1, collider2D *collider2, const std::size_t workload_index)
{
    KIT_PERF_SCOPE("broad_phase2D::process_collision_mt")
    const collision2D colis = generate_collision(collider1, collider2);
    m_mt_metrics[workload_index].total_collision_checks++;
    if (colis.collided)
    {
        m_mt_collisions[workload_index].push_back(colis);
        m_mt_metrics[workload_index].positive_collision_checks++;
        KIT_ASSERT_ERROR(colis.friction >= 0.f, "Friction must be non-negative: {0}", colis.friction)
        KIT_ASSERT_ERROR(colis.restitution >= 0.f, "Restitution must be non-negative: {0}", colis.restitution)
    }
}
void broad_phase2D::join_mt_collisions()
{
    for (std::size_t i = 0; i < params.parallel_workloads; i++)
    {
        const auto &pairs = m_mt_collisions[i];
        const metrics &m = m_mt_metrics[i];
        m_collisions.insert(m_collisions.end(), pairs.begin(), pairs.end());
        m_metrics.total_collision_checks += m.total_collision_checks;
        m_metrics.positive_collision_checks += m.positive_collision_checks;
    }
}

static bool elligible_for_collision(const collider2D *collider1, const collider2D *collider2)
{
    KIT_PERF_SCOPE("broad_phase2D::elligible_for_collision")
    const body2D *body1 = collider1->body();
    const body2D *body2 = collider2->body();
    return body1 != body2 && (body1->is_dynamic() || body2->is_dynamic()) && (!body1->asleep() || !body2->asleep()) &&
           (collider1->collision_filter.cgroups & collider2->collision_filter.collides_with) &&
           (collider2->collision_filter.cgroups & collider1->collision_filter.collides_with) &&
           geo::intersects(collider1->shape().bounding_box(), collider2->shape().bounding_box()) &&
           !body1->joint_prevents_collision(body2);
}

collision2D broad_phase2D::generate_collision(collider2D *collider1, collider2D *collider2) const
{
    KIT_PERF_SCOPE("broad_phase2D::generate_collision")
    collision2D collision;
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

void broad_phase2D::cc_narrow_collision_check(collider2D *collider1, collider2D *collider2,
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
void broad_phase2D::cp_narrow_collision_check(collider2D *collider1, collider2D *collider2,
                                              collision2D &collision) const
{
    const circle &circ = collider1->shape<circle>();
    const polygon &poly = collider2->shape<polygon>();
    const float R = circ.radius() + poly.radius();
    if (glm::distance2(circ.gcentroid(), poly.gcentroid()) > R * R)
        return;

    const narrow_result2D nres = m_cp_narrow->circle_polygon(circ, poly);
    if (!nres)
        return;

    fill_collision_data(collision, collider1, collider2, nres.mtv, nres.manifold);
}
void broad_phase2D::pp_narrow_collision_check(collider2D *collider1, collider2D *collider2,
                                              collision2D &collision) const
{
    const polygon &poly1 = collider1->shape<polygon>();
    const polygon &poly2 = collider2->shape<polygon>();
    const float R = poly1.radius() + poly2.radius();
    if (glm::distance2(poly1.gcentroid(), poly2.gcentroid()) > R * R)
        return;

    const narrow_result2D nres = m_pp_narrow->polygon_polygon(poly1, poly2);
    if (!nres)
        return;

    fill_collision_data(collision, collider1, collider2, nres.mtv, nres.manifold);
}

} // namespace ppx