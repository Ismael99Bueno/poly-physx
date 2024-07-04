#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/quad_tree_broad2D.hpp"
#include "ppx/world2D.hpp"

#include "kit/multithreading/mt_for_each.hpp"

namespace ppx
{
quad_tree_broad2D::quad_tree_broad2D(world2D &world) : broad_phase2D(world)
{
    build_tree_from_scratch();
}
quad_tree_broad2D::~quad_tree_broad2D()
{
    for (collider2D *collider : world.colliders)
        collider->bound();
}

void quad_tree_broad2D::insert(collider2D *collider)
{
    m_needs_partition_update = true;
    if (m_qt_bounds.contains(collider->bounding_box()))
    {
        m_may_rebuild = true;
        m_quad_tree.insert(qt_element{collider});
    }
    else
        build_tree_from_scratch();
}
void quad_tree_broad2D::erase(collider2D *collider)
{
    m_may_rebuild = true;
    m_needs_partition_update = true;
    m_quad_tree.erase(qt_element{collider});
}

void quad_tree_broad2D::detect_collisions()
{
    KIT_PERF_SCOPE("quad_tree_broad2D::detect_collisions")
    // build_tree_from_scratch();
    m_rebuild_timer += world.rk_substep_timestep();
    if (m_may_rebuild && m_rebuild_timer > rebuild_time_threshold)
        build_tree_from_scratch();

    if (m_needs_partition_update)
    {
        m_partitions = m_quad_tree.collect_partitions();
        m_needs_partition_update = false;
    }

    if (params.multithreaded && world.thread_pool)
        detect_collisions_mt();
    else
        detect_collisions_st();
}

void quad_tree_broad2D::detect_collisions_st()
{
    KIT_PERF_SCOPE("quad_tree_broad2D::detect_collisions_st")
    for (const qtpartition &partition : m_partitions)
        for (std::size_t i = 0; i < partition.elements->size(); i++)
        {
            for (std::size_t j = i + 1; j < partition.elements->size(); j++)
            {
                collider2D *collider1 = (*partition.elements)[i].collider;
                collider2D *collider2 = (*partition.elements)[j].collider;
                process_collision_st(collider1, collider2);
            }
            for (const qt_element &qtelm : partition.to_compare)
            {
                collider2D *collider1 = (*partition.elements)[i].collider;
                collider2D *collider2 = qtelm.collider;
                process_collision_st(collider1, collider2);
            }
        }
}
void quad_tree_broad2D::detect_collisions_mt()
{
    kit::mt::for_each(
        *world.thread_pool, m_partitions,
        [this](const std::size_t workload_index, const qtpartition &partition) {
            for (std::size_t i = 0; i < partition.elements->size(); i++)
            {
                for (std::size_t j = i + 1; j < partition.elements->size(); j++)
                {
                    collider2D *collider1 = (*partition.elements)[i].collider;
                    collider2D *collider2 = (*partition.elements)[j].collider;
                    process_collision_mt(collider1, collider2, workload_index);
                }
                for (const qt_element &qtelm : partition.to_compare)
                {
                    collider2D *collider1 = (*partition.elements)[i].collider;
                    collider2D *collider2 = qtelm.collider;
                    process_collision_mt(collider1, collider2, workload_index);
                }
            }
        },
        params.parallel_workloads);
    join_mt_collisions();
}
void quad_tree_broad2D::build_tree_from_scratch()
{
    KIT_PERF_FUNCTION()
    KIT_ASSERT_ERROR(bounding_box_anticipation >= 0.f, "Expansion margin must be non-negative")

    m_may_rebuild = false;
    m_rebuild_timer = 0.f;
    m_rebuild_count++;

    m_quad_tree.clear();
    if (world.colliders.empty())
        return;

    const float expansion_margin = 2.f * bounding_box_anticipation;
    m_qt_bounds = world.colliders[0]->bounding_box();
    for (body2D *body : world.bodies)
    {
        const glm::vec2 enlargement = enlargement_vector_from_velocity(body->velocity());
        for (collider2D *collider : *body)
        {
            collider->bound();
            collider->enlarge_bounding_box(enlargement);
            m_qt_bounds += collider->bounding_box();
        }
    }
    if (force_square_shape)
    {
        const glm::vec2 dim = m_qt_bounds.dimension();
        if (dim.x > dim.y)
        {
            m_qt_bounds.min.y -= 0.5f * (dim.x - dim.y);
            m_qt_bounds.max.y += 0.5f * (dim.x - dim.y);
        }
        else
        {
            m_qt_bounds.min.x -= 0.5f * (dim.y - dim.x);
            m_qt_bounds.max.x += 0.5f * (dim.y - dim.x);
        }
    }
    m_qt_bounds.min -= expansion_margin;
    m_qt_bounds.max += expansion_margin;

    m_quad_tree.bounds(m_qt_bounds);
    for (collider2D *collider : world.colliders)
        m_quad_tree.insert(qt_element{collider});
    KIT_ASSERT_ERROR(m_quad_tree.checksum(), "Quad tree checksum failed")
}

glm::vec2 quad_tree_broad2D::enlargement_vector_from_velocity(const glm::vec2 &velocity) const
{
    return velocity * bounding_box_anticipation;
}
std::uint32_t quad_tree_broad2D::rebuild_count() const
{
    return m_rebuild_count;
}

const ppx::quad_tree &quad_tree_broad2D::quad_tree() const
{
    return m_quad_tree;
}
ppx::quad_tree &quad_tree_broad2D::quad_tree()
{
    return m_quad_tree;
}
} // namespace ppx