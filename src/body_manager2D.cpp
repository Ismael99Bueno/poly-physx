#include "ppx/internal/pch.hpp"
#include "ppx/body_manager2D.hpp"
#include "ppx/world2D.hpp"
#include "geo/intersection.hpp"

namespace ppx
{
body_manager2D::body_manager2D(world2D &world) : m_world(world)
{
}

void body_manager2D::apply_added_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
    {
        body.apply_simulation_force(body.added_force() + body.persistent_force);
        body.apply_simulation_torque(body.added_torque() + body.persistent_torque);
    }
}

void body_manager2D::reset_added_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
        body.reset_added_forces();
}
void body_manager2D::reset_simulation_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
        body.reset_simulation_forces();
}

static std::optional<std::size_t> index_from_id(const kit::uuid id, const std::vector<body2D> &vec)
{
    for (std::size_t i = 0; i < vec.size(); i++)
        if (vec[i].id == id)
            return i;
    return {};
}

body2D::const_ptr body_manager2D::from_id(const kit::uuid id) const
{
    const auto index = index_from_id(id, m_bodies);
    return index ? ptr(index.value()) : nullptr;
}
body2D::ptr body_manager2D::from_id(const kit::uuid id)
{
    const auto index = index_from_id(id, m_bodies);
    return index ? ptr(index.value()) : nullptr;
}

const body2D &body_manager2D::operator[](const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_bodies.size())
    return m_bodies[index];
}
body2D &body_manager2D::operator[](const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_bodies.size())
    return m_bodies[index];
}

body2D::const_ptr body_manager2D::ptr(const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_bodies.size())
    return {&m_bodies, index};
}
body2D::ptr body_manager2D::ptr(const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_bodies.size())
    return {&m_bodies, index};
}

std::vector<body2D::const_ptr> body_manager2D::operator[](const geo::aabb2D &aabb) const
{
    std::vector<body2D::const_ptr> in_area;
    in_area.reserve(m_bodies.size() / 2);

    for (const body2D &body : m_bodies)
        if (geo::intersects(body.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_bodies, body.index);
    return in_area;
}
std::vector<body2D::ptr> body_manager2D::operator[](const geo::aabb2D &aabb)
{
    std::vector<body2D::ptr> in_area;
    in_area.reserve(m_bodies.size() / 2);
    for (const body2D &body : m_bodies)
        if (geo::intersects(body.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_bodies, body.index);
    return in_area;
}

body2D::const_ptr body_manager2D::operator[](const glm::vec2 &point) const
{
    const geo::aabb2D aabb = point;
    for (const body2D &body : m_bodies)
        if (geo::intersects(body.shape().bounding_box(), aabb))
            return {&m_bodies, body.index};
    return nullptr;
}
body2D::ptr body_manager2D::operator[](const glm::vec2 &point)
{
    const geo::aabb2D aabb = point;
    for (const body2D &body : m_bodies)
        if (geo::intersects(body.shape().bounding_box(), aabb))
            return {&m_bodies, body.index};
    return nullptr;
}

body2D::ptr body_manager2D::process_addition(body2D &body)
{
    body.index = m_bodies.size() - 1;
    body.world = &m_world;
    const body2D::ptr e_ptr = {&m_bodies, m_bodies.size() - 1};

    const kit::transform2D &transform = body.transform();
    const glm::vec2 &velocity = body.velocity();

    rk::state &state = m_world.integrator.state;
    state.append({transform.position.x, transform.position.y, transform.rotation, velocity.x, velocity.y,
                  body.angular_velocity()});
    body.retrieve_data_from_state_variables(state.vars());

    KIT_INFO("Added body with index {0} and id {1}.", body.index, (std::uint64_t)body.id)
#ifdef DEBUG
    for (std::size_t i = 0; i < m_bodies.size() - 1; i++)
        KIT_ASSERT_CRITICAL(m_bodies[i].id != body.id, "Body with index {0} has the same id as body with index {1}", i,
                            body.index)
#endif
    m_world.events.on_body_addition(e_ptr);
    return e_ptr;
}

bool body_manager2D::remove(std::size_t index)
{
    if (index >= m_bodies.size())
    {
        KIT_WARN("Body index exceeds array bounds. Aborting... - index: {0}, size: {1}", index, m_bodies.size())
        return false;
    }
    KIT_INFO("Removing body with index {0} and id {1}", index, m_bodies[index].id)

    m_world.events.on_early_body_removal(m_bodies[index]);
    rk::state &state = m_world.integrator.state;
    if (index != m_bodies.size() - 1)
    {
        m_bodies[index] = m_bodies.back();
        m_bodies[index].index = index;
    }
    m_bodies.pop_back();

    for (std::size_t i = 0; i < 6; i++)
        state[6 * index + i] = state[state.size() - 6 + i];
    state.resize(6 * m_bodies.size());

    m_world.validate();
    m_world.events.on_late_body_removal(std::move(index)); // It just made me do this...
    return true;
}

bool body_manager2D::remove(const body2D &body)
{
    return remove(body.index);
}

bool body_manager2D::remove(kit::uuid id)
{
    for (const body2D &body : m_bodies)
        if (body.id == id)
            return remove(body.index);
    return false;
}

void body_manager2D::validate()
{
    std::size_t index = 0;
    for (auto it = m_bodies.begin(); it != m_bodies.end(); index++, ++it)
    {
        it->world = &m_world;
        it->index = index;
    }
}

void body_manager2D::send_data_to_state(rk::state &state)
{
    KIT_PERF_FUNCTION()
    for (const body2D &body : m_bodies)
    {
        const std::size_t index = 6 * body.index;

        const glm::vec2 &position = body.position();
        const glm::vec2 &velocity = body.velocity();
        const float rotation = body.rotation();
        const float angular_velocity = body.angular_velocity();

        state[index] = position.x;
        state[index + 1] = position.y;
        state[index + 2] = rotation;
        state[index + 3] = velocity.x;
        state[index + 4] = velocity.y;
        state[index + 5] = angular_velocity;
    }
}

void body_manager2D::retrieve_data_from_state_variables(const std::vector<float> &vars_buffer)
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
        body.retrieve_data_from_state_variables(vars_buffer);
}

std::size_t body_manager2D::size() const
{
    return m_bodies.size();
}

void body_manager2D::clear()
{
    for (std::size_t i = m_bodies.size() - 1; i < m_bodies.size(); i--)
        remove(i);
}
} // namespace ppx