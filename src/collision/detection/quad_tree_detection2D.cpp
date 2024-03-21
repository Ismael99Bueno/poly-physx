#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/world2D.hpp"

#include "kit/multithreading/mt_for_each.hpp"

namespace ppx
{
void quad_tree_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()
    update_quad_tree();

    const auto partitions = m_quad_tree.collect_partitions();

    if (multithreaded)
        detect_collisions_mt(partitions);
    else
        detect_collisions_st(partitions);
}

void quad_tree_detection2D::detect_collisions_st(const std::vector<const qtpartition *> &partitions)
{
    for (const qtpartition *partition : partitions)
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
                collider2D &collider1 = *(*partition)[i];
                collider2D &collider2 = *(*partition)[j];
                process_collision_st(collider1, collider2);
            }
    // DEBUG COLLISION COUNT CHECK GOES HERE
}
void quad_tree_detection2D::detect_collisions_mt(const std::vector<const qtpartition *> &partitions)
{
    kit::mt::for_each(PPX_THREAD_COUNT, partitions, [this](const std::size_t thread_idx, const qtpartition *partition) {
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
                collider2D &collider1 = *(*partition)[i];
                collider2D &collider2 = *(*partition)[j];
                process_collision_mt(collider1, collider2, thread_idx);
            }
    });
    join_mt_collisions();
}
void quad_tree_detection2D::update_quad_tree()
{
    m_quad_tree.clear();

    aabb2D aabb;
    for (const collider2D &collider : world.colliders)
        if (collider.parent().is_dynamic())
            aabb += collider.bounding_box();

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
    for (collider2D &collider : world.colliders)
        m_quad_tree.insert(&collider,
                           [](const collider2D *collider) -> const geo::aabb2D & { return collider->bounding_box(); });
}

const kit::quad_tree<collider2D *> &quad_tree_detection2D::quad_tree() const
{
    return m_quad_tree;
}
kit::quad_tree<collider2D *> &quad_tree_detection2D::quad_tree()
{
    return m_quad_tree;
}
} // namespace ppx