#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision_manager2D.hpp"

namespace ppx
{
collision_manager2D::collision_manager2D(world2D &world) : m_world(world)
{
    set_detection<quad_tree_detection2D>();
    set_solver<spring_driven_solver2D>();
}

const collision2D &collision_manager2D::operator[](const std::size_t index) const
{
    return m_collision_detection->collisions()[index];
}

void collision_manager2D::solve()
{
    const auto &collisions = m_collision_detection->detect_collisions_cached();
    m_collision_solver->solve(collisions);
}

std::size_t collision_manager2D::size() const
{
    return m_collision_detection->collisions().size();
}
} // namespace ppx