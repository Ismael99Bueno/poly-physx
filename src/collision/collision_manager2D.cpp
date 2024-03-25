#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision_manager2D.hpp"

#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"

#include "ppx/collision/detection/narrow/gjk_epa_detection2D.hpp"

#include "ppx/collision/manifold/radius_distance_manifold2D.hpp"
#include "ppx/collision/manifold/mtv_support_manifold2D.hpp"
#include "ppx/collision/manifold/clipping_algorithm_manifold2D.hpp"

namespace ppx
{
collision_manager2D::collision_manager2D(world2D &world) : worldref2D(world)
{
    set_detection<quad_tree_detection2D>();
    set_resolution<constraint_driven_resolution2D>();

    m_detection->set_cp_narrow_detection<gjk_epa_detection2D>();
    m_detection->set_pp_narrow_detection<gjk_epa_detection2D>();

    m_detection->set_cc_manifold_algorithm<radius_distance_manifold2D>();
    m_detection->set_cp_manifold_algorithm<mtv_support_manifold2D>();
    m_detection->set_pp_manifold_algorithm<clipping_algorithm_manifold2D>();
}

const collision2D &collision_manager2D::operator[](
    const kit::non_commutative_tuple<const collider2D *, const collider2D *> &key) const
{
    return m_detection->collisions().at(key);
}
const collision2D &collision_manager2D::at(
    const kit::non_commutative_tuple<const collider2D *, const collider2D *> &key) const
{
    return m_detection->collisions().at(key);
}
const collision2D &collision_manager2D::at(const collider2D *collider1, const collider2D *collider2) const
{
    return m_detection->collisions().at({collider1, collider2});
}

bool collision_manager2D::contains(const kit::non_commutative_tuple<const collider2D *, const collider2D *> &key) const
{
    return m_detection->collisions().contains(key);
}
bool collision_manager2D::contains(const collider2D *collider1, const collider2D *collider2) const
{
    return m_detection->collisions().contains({collider1, collider2});
}

void collision_manager2D::solve()
{
    KIT_PERF_SCOPE("Collision solve")
    const auto &collisions = m_detection->detect_collisions_cached();
    if (!collisions.empty())
    {
        for (const auto &collision : collisions)
        {
            collision.second.collider1->events.on_collision_pre_solve(collision.second);
            collision.second.collider2->events.on_collision_pre_solve(collision.second);
        }
        m_resolution->solve(collisions);
        for (const auto &collision : collisions)
        {
            collision.second.collider1->events.on_collision_post_solve(collision.second);
            collision.second.collider2->events.on_collision_post_solve(collision.second);
        }
    }
}

std::size_t collision_manager2D::size() const
{
    return m_detection->collisions().size();
}
bool collision_manager2D::empty() const
{
    return m_detection->collisions().empty();
}

} // namespace ppx