#include "ppx/internal/pch.hpp"
#include "ppx/collision/detection/brute_force_detection2D.hpp"
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
const std::vector<collision2D> &brute_force_detection2D::detect_collisions()
{
    KIT_PERF_FUNCTION()
#ifdef PPX_MULTITHREADED
    return detect_collisions_mt();
#else
    return detect_collisions_st();
#endif
}

const std::vector<collision2D> &brute_force_detection2D::detect_collisions_st()
{
    for (std::size_t i = 0; i < m_world->bodies.size(); i++)
        for (std::size_t j = i + 1; j < m_world->bodies.size(); j++)
        {
            collision2D colis;
            const body2D &body1 = m_world->bodies[i];
            const body2D &body2 = m_world->bodies[j];
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
const std::vector<collision2D> &brute_force_detection2D::detect_collisions_mt()
{
    const auto exec = [this](const std::size_t thread_idx, const body2D &body1) {
        for (std::size_t j = 0; j < m_world->bodies.size(); j++)
        {
            collision2D colis;
            const body2D &body2 = m_world->bodies[j];
            if (gather_collision_data(body1, body2, &colis))
            {
                try_enter_or_stay_callback(colis);
                m_mt_collisions[thread_idx].push_back(colis);
            }
            else
                try_exit_callback(body1, body2);
        }
    };
    kit::const_for_each_mt<PPX_THREAD_COUNT, body2D>(m_world->bodies, exec);
    for (const auto &pairs : m_mt_collisions)
        m_collisions.insert(m_collisions.begin(), pairs.begin(), pairs.end());
    return m_collisions;
}
} // namespace ppx