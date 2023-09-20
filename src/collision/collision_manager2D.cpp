#include "ppx/internal/pch.hpp"
#include "ppx/collision/collision_manager2D.hpp"
#include "ppx/world2D.hpp"

#include "geo/intersection.hpp"

#if defined(PPX_MULTITHREADED) && defined(KIT_PROFILE)
#pragma message(                                                                                                       \
        "Multithreading for PPX will be disabled because the thread unsafe profiling features of cpp-kit are enabled")
#undef PPX_MULTITHREADED
#endif

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

collision_manager2D::collision_manager2D(world2D &parent, const std::size_t allocations)
    : m_parent(parent), m_quad_tree({-10.f, -10.f}, {10.f, 10.f})
{
    m_intervals.reserve(allocations);
    m_collision_pairs.reserve(allocations);
#ifdef PPX_MULTITHREADED
    for (auto &v : m_mt_collision_pairs)
        v.reserve(allocations);
#endif
}

collision_manager2D::interval::interval(const body2D::const_ptr &body, const end end_type)
    : m_body(body), m_end(end_type)
{
}

const body2D *collision_manager2D::interval::body() const
{
    return m_body.raw();
}
float collision_manager2D::interval::value() const
{
    const geo::aabb2D &bbox = m_body->shape().bounding_box();
    return (m_end == end::LOWER) ? bbox.min().x : bbox.max().x;
}

collision_manager2D::interval::end collision_manager2D::interval::type() const
{
    return m_end;
}
bool collision_manager2D::interval::valid() const
{
    return (bool)m_body;
}

void collision_manager2D::add_body_intervals(const body2D::const_ptr &body)
{
    m_intervals.emplace_back(body, interval::end::LOWER);
    m_intervals.emplace_back(body, interval::end::HIGHER);
}

void collision_manager2D::solve_and_load_collisions(std::vector<float> &state_derivative)
{
    if (m_collision_pairs.empty())
        broad_and_narrow_fase(state_derivative);
    else
        narrow_fase(state_derivative);
}

void collision_manager2D::broad_and_narrow_fase(std::vector<float> &state_derivative)
{
    KIT_PERF_FUNCTION()
    if (!enabled)
        return;
    switch (m_coldet_method)
    {
    case detection::BRUTE_FORCE:
        brute_force(state_derivative);
        break;
    case detection::SORT_AND_SWEEP:
        sort_and_sweep(state_derivative);
        break;
    case detection::QUAD_TREE:
        quad_tree(state_derivative);
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

void collision_manager2D::narrow_fase(std::vector<float> &state_derivative)
{
#ifdef PPX_MULTITHREADED
    const auto exec = [this, &state_derivative](const std::size_t thread_idx, const colpair &cp) {
        collision2D c;
        if (narrow_detection(*cp.first, *cp.second, &c))
            solve(c, state_derivative);
    };
    for_each_mt(m_collision_pairs, exec);
#else
    for (const colpair &cp : m_collision_pairs)
    {
        collision2D c;
        if (narrow_detection(*cp.first, *cp.second, &c))
            solve(c, state_derivative);
    }
#endif
}

void collision_manager2D::update_quad_tree()
{
    m_quad_tree.clear();
    geo::aabb2D aabb({-10.f, -10.f}, {10.f, 10.f});
    for (const body2D &body : m_parent.bodies())
        aabb += body.shape().bounding_box();

    m_quad_tree.aabb = aabb;
    for (const body2D &body : m_parent.bodies())
        m_quad_tree.insert(&body);
}

void collision_manager2D::validate()
{
    for (auto it = m_intervals.begin(); it != m_intervals.end();)
        if (!it->valid())
            it = m_intervals.erase(it);
        else
            ++it;
}
void collision_manager2D::flush_collisions()
{
    m_collision_pairs.clear();
#ifdef PPX_MULTITHREADED
    for (auto &pairs : m_mt_collision_pairs)
        pairs.clear();
#endif
}

float collision_manager2D::stiffness() const
{
    return m_stiffness;
}
float collision_manager2D::dampening() const
{
    return m_dampening;
}

void collision_manager2D::stiffness(float stiffness)
{
    m_stiffness = stiffness;
}
void collision_manager2D::dampening(float dampening)
{
    m_dampening = dampening;
}

collision_manager2D::detection collision_manager2D::detection_method() const
{
    return m_coldet_method;
}
void collision_manager2D::detection_method(detection coldet)
{
    m_coldet_method = coldet;
}

const quad_tree2D &collision_manager2D::quad_tree() const
{
    return m_quad_tree;
}
quad_tree2D &collision_manager2D::quad_tree()
{
    return m_quad_tree;
}

void collision_manager2D::sort_intervals()
{
    const auto cmp = [](const interval &itrv1, const interval &itrv2) { return itrv1.value() < itrv2.value(); };
    std::sort(m_intervals.begin(), m_intervals.end(), cmp);
}

static bool broad_detection(const body2D &body1, const body2D &body2)
{
    return body1 != body2 && (body1.kinematic || body2.kinematic) && geo::may_intersect(body1.shape(), body2.shape());
}
static bool are_both_circles(const body2D &body1, const body2D &body2)
{
    return body1.type() == body2D::shape_type::CIRCLE && body2.type() == body2D::shape_type::CIRCLE;
}

bool collision_manager2D::narrow_detection_mix(const body2D &body1, const body2D &body2, collision2D *c) const
{
    const geo::shape2D &sh1 = body1.shape(), &sh2 = body2.shape();
    if (!geo::may_intersect(sh1, sh2))
        return false;

    auto simplex = geo::gjk(sh1, sh2);
    if (!simplex)
        return false;

    auto mtv = geo::epa(sh1, sh2, simplex.value());
    if (!mtv)
        return false;

    const auto &[contact1, contact2] = geo::contact_points(sh1, sh2, mtv.value());
    *c = {m_parent[body1.index], m_parent[body2.index], contact1, contact2, mtv.value()};

    return true;
}

bool collision_manager2D::narrow_detection_circle(const body2D &body1, const body2D &body2, collision2D *c) const
{
    const geo::circle &c1 = body1.shape<geo::circle>(), &c2 = body2.shape<geo::circle>();
    if (!geo::intersect(c1, c2))
        return false;
    const glm::vec2 mtv = geo::mtv(c1, c2);
    const auto &[contact1, contact2] = geo::contact_points(c1, c2);
    *c = {m_parent[body1.index], m_parent[body2.index], contact1, contact2, mtv};
    return true;
}

bool collision_manager2D::narrow_detection(const body2D &body1, const body2D &body2, collision2D *c) const
{
    if (are_both_circles(body1, body2))
        return narrow_detection_circle(body1, body2, c);
    return narrow_detection_mix(body1, body2, c);
}
bool collision_manager2D::full_detection(const body2D &body1, const body2D &body2, collision2D *c) const
{
    if (!broad_detection(body1, body2))
        return false;
    return narrow_detection(body1, body2, c);
}

void collision_manager2D::try_enter_or_stay_callback(const collision2D &c) const
{
    c.current->events.try_enter_or_stay(c);
    c.incoming->events.try_enter_or_stay({c.incoming, c.current, c.touch2, c.touch1, -c.normal});
}
void collision_manager2D::try_exit_callback(const body2D &body1, const body2D &body2) const
{
    body1.events.try_exit(m_parent[body1.index], m_parent[body2.index]);
    body2.events.try_exit(m_parent[body2.index], m_parent[body1.index]);
}

void collision_manager2D::brute_force(std::vector<float> &state_derivative)
{
    KIT_PERF_FUNCTION()
#ifdef PPX_MULTITHREADED
    const auto exec = [this, &state_derivative](const std::size_t thread_idx, const body2D &body1) {
        const auto bodies = m_parent.bodies();
        for (std::size_t j = 0; j < m_parent.size(); j++)
        {
            collision2D c;
            const body2D &body2 = bodies[j];
            if (full_detection(body1, body2, &c))
            {
                try_enter_or_stay_callback(c);
                solve(c, state_derivative);
                m_mt_collision_pairs[thread_idx].emplace_back(&body1, &body2);
            }
            else
                try_exit_callback(body1, body2);
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
            const body2D &body1 = bodies[i], &body2 = bodies[j];
            if (full_detection(body1, body2, &c))
            {
#ifdef DEBUG
                collisions++;
#endif
                try_enter_or_stay_callback(c);
                solve(c, state_derivative);
                m_collision_pairs.emplace_back(&body1, &body2);
            }
            else
                try_exit_callback(body1, body2);
        }
    KIT_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for BRUTE FORCE "
              "collision detection (QUALITY: {3:.2f}%%)",
              checks, collisions, checks - collisions, 100.f * (float)m_parent.size() / (float)checks)
#endif
}

void collision_manager2D::sort_and_sweep(std::vector<float> &state_derivative)
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
            for (const body2D *body : eligible)
            {
#ifdef DEBUG
                checks++;
#endif
                collision2D c;
                const body2D &body1 = *body, &body2 = *itrv.body();
                if (full_detection(body1, body2, &c))
                {
#ifdef DEBUG
                    collisions++;
#endif
                    try_enter_or_stay_callback(c);
                    solve(c, state_derivative);
                    m_collision_pairs.emplace_back(&body1, &body2);
                }
                else
                    try_exit_callback(body1, body2);
            }
            eligible.insert(itrv.body());
        }
        else
            eligible.erase(itrv.body());
    KIT_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for SORT AND "
              "SWEEP collision detection (QUALITY: {3:.2f}%%)",
              checks, collisions, checks - collisions, 100.f * (float)m_parent.size() / (float)checks)
}

void collision_manager2D::quad_tree(std::vector<float> &state_derivative)
{
    KIT_PERF_FUNCTION()
    update_quad_tree();

    std::vector<const std::vector<const body2D *> *> partitions;
    partitions.reserve(20);
    m_quad_tree.collect_partitions(partitions);

#ifdef PPX_MULTITHREADED
    const auto exec = [this, &state_derivative](const std::size_t thread_idx,
                                                const std::vector<const body2D *> *partition) {
        for (std::size_t i = 0; i < partition->size(); i++)
            for (std::size_t j = i + 1; j < partition->size(); j++)
            {
                collision2D c;
                const auto &body1 = (*partition)[i], &body2 = (*partition)[j];
                if (full_detection(*body1, *body2, &c))
                {
                    try_enter_or_stay_callback(c);
                    solve(c, state_derivative);
                    m_mt_collision_pairs[thread_idx].emplace_back(body1, body2);
                }
                else
                    try_exit_callback(*body1, *body2);
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
                const auto &body1 = (*partition)[i], &body2 = (*partition)[j];
                if (full_detection(*body1, *body2, &c))
                {
#ifdef DEBUG
                    collisions++;
#endif
                    try_enter_or_stay_callback(c);
                    solve(c, state_derivative);
                    m_collision_pairs.emplace_back(body1, body2);
                }
                else
                    try_exit_callback(*body1, *body2);
            }
    KIT_TRACE("Checked for {0} collisions and solved {1} of them, with a total of {2} false positives for QUAD TREE "
              "collision detection (QUALITY: {3:.2f}%%)",
              checks, collisions, checks - collisions, 100.f * (float)m_parent.size() / (float)checks)
#endif
}

void collision_manager2D::solve(const collision2D &c, std::vector<float> &state_derivative) const
{
    KIT_PERF_FUNCTION()
    const std::array<float, 6> forces = forces_upon_collision(c);
    for (std::size_t i = 0; i < 3; i++)
    {
        if (c.current->kinematic)
            state_derivative[c.current->index * 6 + i + 3] += forces[i];
        if (c.incoming->kinematic)
            state_derivative[c.incoming->index * 6 + i + 3] += forces[i + 3];
    }
}

std::array<float, 6> collision_manager2D::forces_upon_collision(const collision2D &c) const
{
    KIT_PERF_FUNCTION()
    const glm::vec2 rel1 = c.touch1 - c.current->transform().position,
                    rel2 = c.touch2 - c.incoming->transform().position;
    const glm::vec2 vel1 = c.current->velocity_at(rel1), vel2 = c.incoming->velocity_at(rel2);
    const glm::vec2 force = m_stiffness * (c.touch2 - c.touch1) + m_dampening * (vel2 - vel1);

    const float torque1 = cross(rel1, force), torque2 = cross(force, rel2);
    return {force.x, force.y, torque1, -force.x, -force.y, torque2};
}

#ifdef KIT_USE_YAML_CPP
YAML::Node collision_manager2D::serializer::encode(const collision_manager2D &cld) const
{
    YAML::Node node;

    YAML::Node qt = node["Quad tree"];
    qt["Dimensions"] = cld.quad_tree().aabb;
    qt["Max bodies"] = cld.quad_tree().max_bodies;
    qt["Max depth"] = ppx::quad_tree2D::max_depth;
    qt["Min size"] = ppx::quad_tree2D::min_size;

    node["Stiffness"] = cld.stiffness();
    node["Dampening"] = cld.dampening();
    node["Collision detection"] = (int)cld.detection_method();
    node["Enabled"] = cld.enabled;
    return node;
}
bool collision_manager2D::serializer::decode(const YAML::Node &node, collision_manager2D &cld) const
{
    if (!node.IsMap() || node.size() != 5)
        return false;

    const YAML::Node &qt = node["Quad tree"];
    cld.quad_tree().aabb = qt["Dimensions"].as<geo::aabb2D>();
    cld.quad_tree().max_bodies = qt["Max bodies"].as<std::size_t>();
    ppx::quad_tree2D::max_depth = qt["Max depth"].as<std::uint32_t>();
    ppx::quad_tree2D::min_size = qt["Min size"].as<float>();

    cld.stiffness(node["Stiffness"].as<float>());
    cld.dampening(node["Dampening"].as<float>());
    cld.detection_method((ppx::collision_manager2D::detection)node["Collision detection"].as<int>());
    cld.enabled = node["Enabled"].as<bool>();

    return true;
}
#endif
} // namespace ppx