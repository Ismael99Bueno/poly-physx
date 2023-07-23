#include "ppx/internal/pch.hpp"
#include "ppx/collision/collider2D.hpp"
#include "ppx/engine2D.hpp"

#include "geo/intersection.hpp"

#ifdef PPX_MULTITHREADED
#include <execution>
#include <mutex>
#endif

namespace ppx
{
static float cross(const glm::vec2 &v1, const glm::vec2 &v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

collider2D::collider2D(engine2D &parent, const std::size_t allocations)
    : m_parent(parent), m_quad_tree({-10.f, -10.f}, {10.f, 10.f})
{
    m_intervals.reserve(allocations);
    m_collision_pairs.reserve(allocations);
#ifdef PPX_MULTITHREADED
    for (auto &v : m_mt_collision_pairs)
        v.reserve(allocations);
#endif
}

collider2D::interval::interval(const body2D::const_ptr &bd, const end end_type) : m_body(bd), m_end(end_type)
{
}

const body2D *collider2D::interval::body() const
{
    return m_body.raw();
}
float collider2D::interval::value() const
{
    const geo::aabb2D &bbox = m_body->shape().bounding_box();
    return (m_end == end::LOWER) ? bbox.min().x : bbox.max().x;
}

collider2D::interval::end collider2D::interval::type() const
{
    return m_end;
}
bool collider2D::interval::valid() const
{
    return (bool)m_body;
}

void collider2D::add_body_intervals(const body2D::const_ptr &bd)
{
    m_intervals.emplace_back(bd, interval::end::LOWER);
    m_intervals.emplace_back(bd, interval::end::HIGHER);
}

void collider2D::solve_and_load_collisions(std::vector<float> &stchanges)
{
    if (m_collision_pairs.empty())
        broad_and_narrow_fase(stchanges);
    else
        narrow_fase(stchanges);
}

void collider2D::broad_and_narrow_fase(std::vector<float> &stchanges)
{
    KIT_PERF_FUNCTION()
    if (!enabled())
        return;
    switch (m_coldet_method)
    {
    case detection::BRUTE_FORCE:
        brute_force(stchanges);
        break;
    case detection::SORT_AND_SWEEP:
        sort_and_sweep(stchanges);
        break;
    case detection::QUAD_TREE:
        quad_tree(stchanges);
        break;
    }
}

#ifdef PPX_MULTITHREADED
template <typename It, typename Func> static void compute(It it1, It it2, Func func, const std::size_t thread_idx)
{
    for (auto it = it1; it != it2; ++it)
        func(thread_idx, *it);
}

template <template <typename...> typename C, typename T, typename Func>
static void for_each_mt(const C<T> &vec, Func func)
{
    std::array<std::thread, PPX_THREAD_COUNT> threads;
    for (std::size_t i = 0; i < PPX_THREAD_COUNT; i++)
    {
        const std::size_t start = i * vec.size() / PPX_THREAD_COUNT, end = (i + 1) * vec.size() / PPX_THREAD_COUNT;

        KIT_ASSERT_ERROR(end <= vec.size(), "Partition exceeds vector size! start: {0}, end: {1}, size: {2}", start,
                         end, vec.size())
        threads[i] = std::thread(compute<decltype(vec.begin()), Func>, vec.begin() + (long)start,
                                 vec.begin() + (long)end, func, i);
    }
    for (std::thread &th : threads)
    {
        KIT_ASSERT_ERROR(th.joinable(), "Thread is not joinable!")
        th.join();
    }
}
#endif

void collider2D::narrow_fase(std::vector<float> &stchanges)
{
#ifdef PPX_MULTITHREADED
    const auto exec = [this, &stchanges](const std::size_t thread_idx, const colpair &cp) {
        collision2D c;
        if (narrow_detection(*cp.first, *cp.second, &c))
            solve(c, stchanges);
    };
    for_each_mt(m_collision_pairs, exec);
#else
    for (const colpair &cp : m_collision_pairs)
    {
        collision2D c;
        if (narrow_detection(*cp.first, *cp.second, &c))
            solve(c, stchanges);
    }
#endif
}

void collider2D::update_quad_tree()
{
    m_quad_tree.clear();
    geo::aabb2D aabb({-10.f, -10.f}, {10.f, 10.f});
    for (const body2D &bd : m_parent.bodies())
        aabb += bd.shape().bounding_box();

    m_quad_tree.aabb(aabb);
    for (const body2D &bd : m_parent.bodies())
        m_quad_tree.insert(&bd);
}

void collider2D::validate()
{
    for (auto it = m_intervals.begin(); it != m_intervals.end();)
        if (!it->valid())
            it = m_intervals.erase(it);
        else
            ++it;
}
void collider2D::flush_collisions()
{
    m_collision_pairs.clear();
#ifdef PPX_MULTITHREADED
    for (auto &pairs : m_mt_collision_pairs)
        pairs.clear();
#endif
}

float collider2D::stiffness() const
{
    return m_stiffness;
}
float collider2D::dampening() const
{
    return m_dampening;
}

void collider2D::stiffness(float stiffness)
{
    m_stiffness = stiffness;
}
void collider2D::dampening(float dampening)
{
    m_dampening = dampening;
}

collider2D::detection collider2D::detection_method() const
{
    return m_coldet_method;
}
void collider2D::detection_method(detection coldet)
{
    m_coldet_method = coldet;
}

const quad_tree2D &collider2D::quad_tree() const
{
    return m_quad_tree;
}
quad_tree2D &collider2D::quad_tree()
{
    return m_quad_tree;
}

void collider2D::sort_intervals()
{
    const auto cmp = [](const interval &itrv1, const interval &itrv2) { return itrv1.value() < itrv2.value(); };
    std::sort(m_intervals.begin(), m_intervals.end(), cmp);
}

static bool broad_detection(const body2D &bd1, const body2D &bd2)
{
    return bd1 != bd2 && (bd1.kinematic() || bd2.kinematic()) && geo::may_intersect(bd1.shape(), bd2.shape());
}
static bool are_both_circles(const body2D &bd1, const body2D &bd2)
{
    return bd1.type() == body2D::shape_type::CIRCLE && bd2.type() == body2D::shape_type::CIRCLE;
}

bool collider2D::narrow_detection_mix(const body2D &bd1, const body2D &bd2, collision2D *c) const
{
    const geo::shape2D &sh1 = bd1.shape(), &sh2 = bd2.shape();
    if (!geo::may_intersect(sh1, sh2))
        return false;

    auto simplex = geo::gjk(sh1, sh2);
    if (!simplex)
        return false;

    auto mtv = geo::epa(sh1, sh2, simplex.value());
    if (!mtv)
        return false;

    const auto &[contact1, contact2] = geo::contact_points(sh1, sh2, mtv.value());
    *c = {m_parent[bd1.index()], m_parent[bd2.index()], contact1, contact2, mtv.value()};

    return true;
}

bool collider2D::narrow_detection_circle(const body2D &bd1, const body2D &bd2, collision2D *c) const
{
    const geo::circle &c1 = bd1.shape<geo::circle>(), &c2 = bd2.shape<geo::circle>();
    if (!geo::intersect(c1, c2))
        return false;
    const glm::vec2 mtv = geo::mtv(c1, c2);
    const auto &[contact1, contact2] = geo::contact_points(c1, c2);
    *c = {m_parent[bd1.index()], m_parent[bd2.index()], contact1, contact2, mtv};
    return true;
}

bool collider2D::narrow_detection(const body2D &bd1, const body2D &bd2, collision2D *c) const
{
    if (are_both_circles(bd1, bd2))
        return narrow_detection_circle(bd1, bd2, c);
    return narrow_detection_mix(bd1, bd2, c);
}
bool collider2D::full_detection(const body2D &bd1, const body2D &bd2, collision2D *c) const
{
    if (!broad_detection(bd1, bd2))
        return false;
    return narrow_detection(bd1, bd2, c);
}

void collider2D::try_enter_or_stay_callback(const body2D &bd1, const body2D &bd2, const collision2D &c) const
{
    bd1.events().try_enter_or_stay(c);
    bd2.events().try_enter_or_stay({c.incoming, c.current, c.touch2, c.touch1, -c.normal});
}
void collider2D::try_exit_callback(const body2D &bd1, const body2D &bd2) const
{
    bd1.events().try_exit(m_parent[bd2.index()]);
    bd2.events().try_exit(m_parent[bd1.index()]);
}

void collider2D::brute_force(std::vector<float> &stchanges)
{
    KIT_PERF_FUNCTION()
#ifdef PPX_MULTITHREADED
    const auto exec = [this, &stchanges](const std::size_t thread_idx, const body2D &bd1) {
        const auto bodies = m_parent.bodies();
        for (std::size_t j = 0; j < m_parent.size(); j++)
        {
            collision2D c;
            const body2D &bd2 = bodies[j];
            if (full_detection(bd1, bd2, &c))
            {
                try_enter_or_stay_callback(bd1, bd2, c);
                solve(c, stchanges);
                m_mt_collision_pairs[thread_idx].emplace_back(&bd1, &bd2);
            }
            else
                try_exit_callback(bd1, bd2);
        }
    };
    for_each_mt(m_parent.bodies().unwrap(), exec);
    for (const auto &pairs : m_mt_collision_pairs)
        m_collision_pairs.insert(m_collision_pairs.begin(), pairs.begin(), pairs.end());
#else
#ifdef DEBUG
    std::size_t checks = 0, collisions = 0;
#endif
    const auto bodies = m_parent.bodies();
    for (std::size_t i = 0; i < m_parent.size(); i++)
        for (std::size_t j = i + 1; j < m_parent.size(); j++)
        {
#ifdef DEBUG
            checks++;
#endif
            collision2D c;
            const body2D &bd1 = bodies[i], &bd2 = bodies[j];
            if (full_detection(bd1, bd2, &c))
            {
#ifdef DEBUG
                collisions++;
#endif
                try_enter_or_stay_callback(bd1, bd2, c);
                solve(c, stchanges);
                m_collision_pairs.emplace_back(&bd1, &bd2);
            }
            else
                try_exit_callback(bd1, bd2);
        }
    KIT_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for BRUTE FORCE "
              "collision detection (QUALITY: {3:.2f}%%)",
              checks, collisions, checks - collisions, 100.f * (float)m_parent.size() / (float)checks)
#endif
}

void collider2D::sort_and_sweep(std::vector<float> &stchanges)
{
    KIT_PERF_FUNCTION()
#ifdef DEBUG
    std::size_t checks = 0, collisions = 0;
#endif
    std::unordered_set<const body2D *> eligible;
    sort_intervals();

    eligible.reserve(30);
    for (const interval &itrv : m_intervals)
        if (itrv.type() == interval::end::LOWER)
        {
            for (const body2D *bd : eligible)
            {
#ifdef DEBUG
                checks++;
#endif
                collision2D c;
                const body2D &bd1 = *bd, &bd2 = *itrv.body();
                if (full_detection(bd1, bd2, &c))
                {
#ifdef DEBUG
                    collisions++;
#endif
                    try_enter_or_stay_callback(bd1, bd2, c);
                    solve(c, stchanges);
                    m_collision_pairs.emplace_back(&bd1, &bd2);
                }
                else
                    try_exit_callback(bd1, bd2);
            }
            eligible.insert(itrv.body());
        }
        else
            eligible.erase(itrv.body());
    KIT_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for SORT AND "
              "SWEEP collision detection (QUALITY: {3:.2f}%%)",
              checks, collisions, checks - collisions, 100.f * (float)m_parent.size() / (float)checks)
}

void collider2D::quad_tree(std::vector<float> &stchanges)
{
    KIT_PERF_FUNCTION()
    update_quad_tree();

    std::vector<const std::vector<const body2D *> *> partitions;
    partitions.reserve(20);
    m_quad_tree.partitions(partitions);

#ifdef PPX_MULTITHREADED
    const auto exec = [this, &stchanges](const std::size_t thread_idx, const std::vector<const body2D *> *partition) {
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
                collision2D c;
                const auto &bd1 = (*partition)[i], &bd2 = (*partition)[j];
                if (full_detection(*bd1, *bd2, &c))
                {
                    try_enter_or_stay_callback(*bd1, *bd2, c);
                    solve(c, stchanges);
                    m_mt_collision_pairs[thread_idx].emplace_back(bd1, bd2);
                }
                else
                    try_exit_callback(*bd1, *bd2);
            }
    };
    for_each_mt(partitions, exec);
    for (const auto &pairs : m_mt_collision_pairs)
        m_collision_pairs.insert(m_collision_pairs.begin(), pairs.begin(), pairs.end());
#else
#ifdef DEBUG
    std::size_t checks = 0, collisions = 0;
#endif
    for (const std::vector<const body2D *> *partition : partitions)
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
#ifdef DEBUG
                checks++;
#endif
                collision2D c;
                const auto &bd1 = (*partition)[i], &bd2 = (*partition)[j];
                if (full_detection(*bd1, *bd2, &c))
                {
#ifdef DEBUG
                    collisions++;
#endif
                    try_enter_or_stay_callback(*bd1, *bd2, c);
                    solve(c, stchanges);
                    m_collision_pairs.emplace_back(bd1, bd2);
                }
                else
                    try_exit_callback(*bd1, *bd2);
            }
    KIT_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for QUAD TREE "
              "collision detection (QUALITY: {3:.2f}%%)",
              checks, collisions, checks - collisions, 100.f * (float)m_parent.size() / (float)checks)
#endif
}

void collider2D::solve(const collision2D &c, std::vector<float> &stchanges) const
{
    KIT_PERF_FUNCTION()
    const std::array<float, 6> forces = forces_upon_collision(c);
    for (std::size_t i = 0; i < 3; i++)
    {
        if (c.current->kinematic())
            stchanges[c.current->index() * 6 + i + 3] += forces[i];
        if (c.incoming->kinematic())
            stchanges[c.incoming->index() * 6 + i + 3] += forces[i + 3];
    }
}

std::array<float, 6> collider2D::forces_upon_collision(const collision2D &c) const
{
    KIT_PERF_FUNCTION()
    const glm::vec2 rel1 = c.touch1 - c.current->position(), rel2 = c.touch2 - c.incoming->position();

    const glm::vec2 vel1 = c.current->vel_at(rel1), vel2 = c.incoming->vel_at(rel2);

    const glm::vec2 force = m_stiffness * (c.touch2 - c.touch1) + m_dampening * (vel2 - vel1);

    const float torque1 = cross(rel1, force), torque2 = cross(force, rel2);
    return {force.x, force.y, torque1, -force.x, -force.y, torque2};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node collider2D::serializer::encode(const collider2D &cld) const
{
    YAML::Node node;

    YAML::Node qt = node["Quad tree"];
    qt["Dimensions"] = cld.quad_tree().aabb();
    qt["Max bodies"] = cld.quad_tree().max_bodies();
    qt["Max depth"] = ppx::quad_tree2D::max_depth();
    qt["Min size"] = ppx::quad_tree2D::min_size();

    node["Stiffness"] = cld.stiffness();
    node["Dampening"] = cld.dampening();
    node["Collision detection"] = (int)cld.detection_method();
    node["Enabled"] = cld.enabled();
    return node;
}
bool collider2D::serializer::decode(const YAML::Node &node, collider2D &cld) const
{
    if (!node.IsMap() || node.size() != 5)
        return false;

    const YAML::Node &qt = node["Quad tree"];
    cld.quad_tree().aabb(qt["Dimensions"].as<geo::aabb2D>());
    cld.quad_tree().max_bodies(qt["Max bodies"].as<std::size_t>());
    ppx::quad_tree2D::max_depth(qt["Max depth"].as<std::uint32_t>());
    ppx::quad_tree2D::min_size(qt["Min size"].as<float>());

    cld.stiffness(node["Stiffness"].as<float>());
    cld.dampening(node["Dampening"].as<float>());
    cld.detection_method((ppx::collider2D::detection)node["Collision detection"].as<int>());
    cld.enabled(node["Enabled"].as<bool>());

    return true;
}
#endif
} // namespace ppx