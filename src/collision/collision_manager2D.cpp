#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision_manager2D.hpp"

namespace ppx
{
collision_manager2D::collision_manager2D(world2D &world) : m_world(world)
{
    set_detection<quad_tree_detection2D>();
    set_solver<spring_solver2D>();
}

void collision_manager2D::solve()
{
    const auto &collisions = m_collision_detection->cached_collisions();
    m_collision_solver->solve(collisions);
}
} // namespace ppx