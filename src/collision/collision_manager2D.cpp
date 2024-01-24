#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision_manager2D.hpp"

#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"

#include "ppx/collision/manifold/radius_distance_manifold2D.hpp"
#include "ppx/collision/manifold/mtv_support_manifold2D.hpp"
#include "ppx/collision/manifold/clipping_algorithm_manifold2D.hpp"

namespace ppx
{
collision_manager2D::collision_manager2D(world2D &world) : worldref2D(world)
{
    set_detection<quad_tree_detection2D>();
    set_resolution<constraint_driven_resolution2D>();

    m_collision_detection->set_cc_manifold_algorithm<radius_distance_manifold2D>();
    m_collision_detection->set_cp_manifold_algorithm<mtv_support_manifold2D>();
    m_collision_detection->set_pp_manifold_algorithm<clipping_algorithm_manifold2D>();
}

const collision2D &collision_manager2D::operator[](const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_collision_detection->collisions().size(), "Index exceeds container size: {0}", index)
    return m_collision_detection->collisions()[index];
}

void collision_manager2D::solve()
{
    const auto &collisions = m_collision_detection->detect_collisions_cached();
    m_collision_resolution->solve(collisions);
}

std::size_t collision_manager2D::size() const
{
    return m_collision_detection->collisions().size();
}

} // namespace ppx