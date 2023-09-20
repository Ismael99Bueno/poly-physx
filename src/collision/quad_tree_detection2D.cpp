#include "ppx/internal/pch.hpp"
#include "ppx/collision/quad_tree_detection2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/profile/perf.hpp"
#include "kit/utility/multithreading.hpp"

#if defined(PPX_MULTITHREADED) && defined(KIT_PROFILE)
#pragma message(                                                                                                       \
        "Multithreading for PPX will be disabled because the thread unsafe profiling features of cpp-kit are enabled")
#undef PPX_MULTITHREADED
#endif

namespace ppx
{
const std::vector<collision2D> &quad_tree_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()
    update_quad_tree();

    std::vector<const quad_tree2D::partition *> partitions;
    partitions.reserve(20);
    m_quad_tree.collect_partitions(partitions);

#ifdef PPX_MULTITHREADED
    return detect_collisions_mt(partitions);
#else
    return detect_collisions_st(partitions);
#endif
}

const std::vector<collision2D> &quad_tree_detection2D::detect_collisions_st(
    const std::vector<const quad_tree2D::partition *> &partitions)
{
    for (const quad_tree2D::partition *partition : partitions)
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
                collision2D colis;
                const body2D &body1 = *(*partition)[i];
                const body2D &body2 = *(*partition)[j];
                if (gather_collision_data(body1, body2, &colis))
                {
                    try_enter_or_stay_callback(colis);
                    m_collisions.push_back(colis);
                }
                else
                    try_exit_callback(body1, body2);
            }
    // DEBUG COLLISION COUNT CHECK GOES HERE
    return m_collisions;
}
const std::vector<collision2D> &quad_tree_detection2D::detect_collisions_mt(
    const std::vector<const quad_tree2D::partition *> &partitions)
{
    const auto exec = [this](const std::size_t thread_idx, const quad_tree2D::partition *partition) {
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
                collision2D colis;
                const body2D &body1 = *(*partition)[i];
                const body2D &body2 = *(*partition)[j];
                if (gather_collision_data(body1, body2, &colis))
                {
                    try_enter_or_stay_callback(colis);
                    m_mt_collisions[thread_idx].push_back(colis);
                }
                else
                    try_exit_callback(body1, body2);
            }
    };
    kit::const_for_each_mt<PPX_THREAD_COUNT, const quad_tree2D::partition *>(partitions, exec);
    for (const auto &pairs : m_mt_collisions)
        m_collisions.insert(m_collisions.end(), pairs.begin(), pairs.end());
    return m_collisions;
}

void quad_tree_detection2D::update_quad_tree()
{
    m_quad_tree.clear();
    geo::aabb2D aabb({-10.f, -10.f}, {10.f, 10.f});
    for (const body2D &body : m_parent.bodies())
        aabb += body.shape().bounding_box();

    m_quad_tree.aabb = aabb;
    for (const body2D &body : m_parent.bodies())
        m_quad_tree.insert(&body);
}
} // namespace ppx