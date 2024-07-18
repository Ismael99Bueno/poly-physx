#include "ppx/internal/pch.hpp"
#include "ppx/collision/broad/quad_tree_broad2D.hpp"
#include "ppx/world2D.hpp"

#include "kit/multithreading/mt_for_each.hpp"

namespace ppx
{

const char *quad_tree_broad2D::name() const
{
    return "Quad Tree";
}

void quad_tree_broad2D::insert(collider2D *collider)
{
    KIT_PERF_SCOPE("ppx::quad_tree_broad2D::insert")
    if (geo::intersects(collider->fat_bbox(), m_qt_bounds))
    {
        m_may_rebuild = true;
        m_quad_tree.insert(collider, collider->fat_bbox());
    }
    else
        build_tree_from_scratch();
}
void quad_tree_broad2D::erase(collider2D *collider)
{
    KIT_PERF_SCOPE("ppx::quad_tree_broad2D::erase")
    m_may_rebuild = true;
    m_quad_tree.erase(collider, collider->fat_bbox());
}

void quad_tree_broad2D::update_pairs(const std::vector<collider2D *> &to_update)
{
    m_rebuild_timer += world.rk_substep_timestep();
    if (m_may_rebuild && m_rebuild_timer > rebuild_time_threshold)
        build_tree_from_scratch();

    if (params.multithreading && world.thread_pool)
        update_pairs_mt(to_update);
    else
        update_pairs_st(to_update);
}

void quad_tree_broad2D::update_pairs_st(const std::vector<collider2D *> &to_update)
{
    KIT_PERF_SCOPE("ppx::quad_tree_broad2D::update_pairs_st")
    for (collider2D *collider1 : to_update)
        m_quad_tree.traverse(
            [this, collider1](collider2D *collider2) {
                collider2D *c1 = collider1;
                if (is_potential_new_pair(&c1, &collider2))
                {
                    m_pairs.emplace_back(c1, collider2);
                    m_unique_pairs.emplace(c1, collider2);
                }
                return true;
            },
            collider1->fat_bbox());
}
void quad_tree_broad2D::update_pairs_mt(const std::vector<collider2D *> &to_update)
{
    KIT_PERF_SCOPE("ppx::quad_tree_broad2D::update_pairs_mt")
    auto pool = world.thread_pool;

    const auto lambda = [this](auto it1, auto it2) {
        thread_local std::vector<pair> new_pairs;
        thread_local std::unordered_set<ctuple> unique_pairs;
        new_pairs.clear();
        unique_pairs.clear();

        for (auto it = it1; it != it2; ++it)
            m_quad_tree.traverse(
                [this, &it](collider2D *collider2) {
                    collider2D *c1 = *it;
                    if (is_potential_new_pair(&c1, &collider2) && unique_pairs.emplace(c1, collider2).second)
                        new_pairs.emplace_back(c1, collider2);
                    return true;
                },
                (*it)->fat_bbox());
        return new_pairs;
    };
    auto futures = kit::mt::for_each_iter(*pool, to_update, lambda, pool->thread_count());
    const std::size_t start_idx = m_pairs.size();
    for (auto &f : futures)
    {
        const auto new_pairs = f.get();
        m_pairs.insert(m_pairs.end(), new_pairs.begin(), new_pairs.end());
    }
    for (auto it = m_pairs.begin() + start_idx; it != m_pairs.end(); ++it)
        m_unique_pairs.emplace(it->collider1, it->collider2);
}
void quad_tree_broad2D::build_tree_from_scratch()
{
    KIT_PERF_SCOPE("ppx::quad_tree_broad2D::build_tree_from_scratch")

    m_may_rebuild = false;
    m_rebuild_timer = 0.f;
    m_rebuild_count++;

    m_quad_tree.clear();
    if (world.colliders.empty())
        return;

    const float expansion_margin = 2.f * world.colliders.params.bbox_enlargement;
    m_qt_bounds = world.colliders[0]->fat_bbox();
    for (collider2D *collider : world.colliders)
        m_qt_bounds += collider->fat_bbox();

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

    m_quad_tree.aabb(m_qt_bounds);
    for (collider2D *collider : world.colliders)
        m_quad_tree.insert(collider, collider->fat_bbox());
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

const ppx::quad_tree::properties &quad_tree_broad2D::props() const
{
    return m_quad_tree.props();
}
void quad_tree_broad2D::props(const ppx::quad_tree::properties &props)
{
    m_quad_tree.clear();
    m_quad_tree.props(props);
    build_tree_from_scratch();
}

} // namespace ppx