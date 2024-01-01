#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/profile/perf.hpp"
#include "kit/utility/multithreading.hpp"

namespace ppx
{
void quad_tree_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()
    update_quad_tree();

    std::vector<const quad_tree::partition *> partitions;
    partitions.reserve(20);
    m_quad_tree.collect_partitions(partitions);

    if (multithreaded)
        detect_collisions_mt(partitions);
    else
        detect_collisions_st(partitions);
}

void quad_tree_detection2D::detect_collisions_st(const std::vector<const quad_tree::partition *> &partitions)
{
    for (const quad_tree::partition *partition : partitions)
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
                body2D &body1 = *(*partition)[i];
                body2D &body2 = *(*partition)[j];
                const collision2D colis = generate_collision(body1, body2);
                if (colis.collided)
                {
                    try_enter_or_stay_callback(colis);
                    m_collisions.push_back(colis);
                }
                else
                    try_exit_callback(body1, body2);
            }
    // DEBUG COLLISION COUNT CHECK GOES HERE
}
void quad_tree_detection2D::detect_collisions_mt(const std::vector<const quad_tree::partition *> &partitions)
{
    const auto exec = [this](const std::size_t thread_idx, const quad_tree::partition *partition) {
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
                body2D &body1 = *(*partition)[i];
                body2D &body2 = *(*partition)[j];
                const collision2D colis = generate_collision(body1, body2);
                if (colis.collided)
                {
                    try_enter_or_stay_callback(colis);
                    m_mt_collisions[thread_idx].push_back(colis);
                }
                else
                    try_exit_callback(body1, body2);
            }
    };
    kit::for_each_mt<PPX_THREAD_COUNT>(partitions, exec);
    for (const auto &pairs : m_mt_collisions)
        m_collisions.insert(m_collisions.end(), pairs.begin(), pairs.end());
}
void quad_tree_detection2D::update_quad_tree()
{
    m_quad_tree.clear();
    geo::aabb2D aabb({-10.f, -10.f}, {10.f, 10.f});
    for (const body2D &body : world->bodies)
        if (body.kinematic)
            aabb += body.shape().bounding_box();

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

    m_quad_tree.aabb = aabb;
    for (body2D &body : world->bodies)
        m_quad_tree.insert(&body);
}

const quad_tree &quad_tree_detection2D::qtree() const
{
    return m_quad_tree;
}
} // namespace ppx