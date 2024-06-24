#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/quad_tree_broad2D.hpp"
#include "ppx/world2D.hpp"

#include "kit/multithreading/mt_for_each.hpp"

namespace ppx
{
void quad_tree_broad2D::detect_collisions()
{
    KIT_PERF_SCOPE("quad_tree_broad2D::detect_collisions")
    update_quad_tree();

    const auto partitions = m_quad_tree.collect_partitions();

    if (params.multithreaded)
        detect_collisions_mt(partitions);
    else
        detect_collisions_st(partitions);
}

void quad_tree_broad2D::detect_collisions_st(const std::vector<qtpartition> &partitions)
{
    KIT_PERF_SCOPE("quad_tree_broad2D::detect_collisions_st")
    for (const qtpartition &partition : partitions)
        for (std::size_t i = 0; i < partition.elements->size(); i++)
        {
            for (std::size_t j = i + 1; j < partition.elements->size(); j++)
            {
                collider2D *collider1 = (*partition.elements)[i];
                collider2D *collider2 = (*partition.elements)[j];
                process_collision_st(collider1, collider2);
            }
#ifdef KIT_QT_COLLECT_ELEMENTS_COPY
            for (collider2D *collider2 : partition.to_compare)
            {
                collider2D *collider1 = (*partition.elements)[i];
                process_collision_st(collider1, collider2);
            }
#else
            for (const auto &elems : partition.to_compare)
                for (collider2D *collider2 : *elems)
                {
                    collider2D *collider1 = (*partition.elements)[i];
                    process_collision_st(collider1, collider2);
                }
#endif
        }
    // DEBUG COLLISION COUNT CHECK GOES HERE
}
void quad_tree_broad2D::detect_collisions_mt(const std::vector<qtpartition> &partitions)
{
    kit::mt::for_each<PPX_THREAD_COUNT>(partitions, [this](const std::size_t thread_idx, const qtpartition &partition) {
        for (std::size_t i = 0; i < partition.elements->size(); i++)
        {
            for (std::size_t j = i + 1; j < partition.elements->size(); j++)
            {
                collider2D *collider1 = (*partition.elements)[i];
                collider2D *collider2 = (*partition.elements)[j];
                process_collision_mt(collider1, collider2, thread_idx);
            }
#ifdef KIT_QT_COLLECT_ELEMENTS_COPY
            for (collider2D *collider2 : partition.to_compare)
            {
                collider2D *collider1 = (*partition.elements)[i];
                process_collision_mt(collider1, collider2, thread_idx);
            }
#else
            for (const auto &elems : partition.to_compare)
                for (collider2D *collider2 : *elems)
                {
                    collider2D *collider1 = (*partition.elements)[i];
                    process_collision_mt(collider1, collider2, thread_idx);
                }
#endif
        }
    });
    join_mt_collisions();
}
void quad_tree_broad2D::update_quad_tree()
{
    KIT_PERF_FUNCTION()
    m_quad_tree.clear();

    aabb2D aabb;
    bool first = true;
    for (const collider2D *collider : world.colliders)
    {
        if (!include_non_dynamic && !collider->body()->is_dynamic())
            continue;

        if (first)
        {
            aabb = collider->bounding_box();
            first = false;
        }
        else
            aabb += collider->bounding_box();
    }

    if (force_square_shape)
    {
        const glm::vec2 dim = aabb.dimension();
        if (dim.x > dim.y)
        {
            aabb.min.y -= 0.5f * (dim.x - dim.y);
            aabb.max.y += 0.5f * (dim.x - dim.y);
        }
        else
        {
            aabb.min.x -= 0.5f * (dim.y - dim.x);
            aabb.max.x += 0.5f * (dim.y - dim.x);
        }
    }

    m_quad_tree.bounds(aabb);
    for (collider2D *collider : world.colliders)
        m_quad_tree.insert(collider,
                           [](const collider2D *collider) -> const geo::aabb2D & { return collider->bounding_box(); });
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