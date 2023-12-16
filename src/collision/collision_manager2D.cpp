#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision_manager2D.hpp"

#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/collision/detection/sort_sweep_detection2D.hpp"
#include "ppx/collision/detection/brute_force_detection2D.hpp"

#include "ppx/collision/solvers/spring_driven_solver2D.hpp"
#include "ppx/collision/solvers/constraint_driven_solver2D.hpp"

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

collision_manager2D::detection_type collision_manager2D::detection_method() const
{
    return m_det_type;
}
collision_manager2D::solver_type collision_manager2D::solver_method() const
{
    return m_solv_type;
}

collision_detection2D *collision_manager2D::detection(const detection_type det_type)
{
    switch (det_type)
    {
    case detection_type::QUAD_TREE:
        return dynamic_cast<collision_detection2D *>(set_detection<quad_tree_detection2D>());
    case detection_type::SORT_AND_SWEEP:
        return dynamic_cast<collision_detection2D *>(set_detection<sort_sweep_detection2D>());
    case detection_type::BRUTE_FORCE:
        return dynamic_cast<collision_detection2D *>(set_detection<brute_force_detection2D>());
    case detection_type::CUSTOM:
        return nullptr;
    }
}
collision_solver2D *collision_manager2D::solver(const solver_type solv_type)
{
    switch (solv_type)
    {
    case solver_type::SPRING_DRIVEN:
        return dynamic_cast<collision_solver2D *>(set_solver<spring_driven_solver2D>());
    case solver_type::CONSTRAINT_DRIVEN:
        return dynamic_cast<collision_solver2D *>(set_solver<constraint_driven_solver2D>());
    case solver_type::CUSTOM:
        return nullptr;
    }
}
} // namespace ppx