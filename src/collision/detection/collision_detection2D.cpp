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

void collision_detection2D::query_last_contact_points()
{
    for (collision2D &col : m_collisions)
    {
        const two_id_hash hash = {col.current->id, col.incoming->id};
        if (m_last_contacts.find(hash) != m_last_contacts.end())
        {
            contact_point_query &cpq = m_last_contacts.at(hash);
            for (std::size_t i = 0; i < cpq.size; i++)
                col.add_contact_point(cpq.contacts[i]);
            cpq.add_contact_point(col.manifold[0]);
            cpq.lifetime = contact_point_query::MAX_LIFETIME;
        }
        else
            m_last_contacts.emplace(std::make_pair(hash, contact_point_query(col.manifold[0])));
    }
    for (auto it = m_last_contacts.begin(); it != m_last_contacts.end();)
        if (it->second.lifetime-- == 0)
            it = m_last_contacts.erase(it);
        else
            ++it;
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
    const glm::vec2 mtv = geo::mtv(c1, c2);
    *colis = {body1.as_ptr(), body2.as_ptr(), mtv, {geo::contact_point(c1, c2)}};
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

    const geo::epa_result epres = geo::epa(sh1, sh2, gres.simplex);
    if (!epres.valid)
        return false;

    *colis = {body1.as_ptr(), body2.as_ptr(), epres.mtv, {geo::contact_point(sh1, sh2, epres.mtv)}};

    return true;
}

void collision_detection2D::try_enter_or_stay_callback(const collision2D &c) const
{
    c.current->events.try_enter_or_stay(c);
    c.incoming->events.try_enter_or_stay(c.reciprocal());
}
void collision_detection2D::try_exit_callback(body2D &body1, body2D &body2) const
{
    body1.events.try_exit(body1.as_ptr(), body2.as_ptr());
    body2.events.try_exit(body2.as_ptr(), body1.as_ptr());
}

} // namespace ppx

std::size_t std::hash<ppx::two_id_hash>::operator()(const ppx::two_id_hash &key) const
{
    const std::uint64_t a = (std::uint64_t)key.id1;
    const std::uint64_t b = (std::uint64_t)key.id2;
    return std::hash<uint64_t>()(a >= b ? a * a + a + b : b * b + b + a);

} // namespace std