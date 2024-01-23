#include "ppx/internal/pch.hpp"
#include "ppx/entities/body_manager2D.hpp"
#include "ppx/world2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
body_manager2D::body_manager2D(world2D &world) : world(world)
{
}

void body_manager2D::apply_impulse_and_persistent_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
    {
        body.apply_simulation_force(body.impulse_force + body.persistent_force);
        body.apply_simulation_torque(body.impulse_torque + body.persistent_torque);
    }
}

void body_manager2D::reset_impulse_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
    {
        body.impulse_force = glm::vec2(0.f);
        body.impulse_torque = 0.f;
    }
}
void body_manager2D::reset_simulation_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
        body.reset_simulation_forces();
}

const body2D *body_manager2D::operator[](const kit::uuid id) const
{
    for (const body2D &body : m_bodies)
        if (body.id == id)
            return &body;
    return nullptr;
}
body2D *body_manager2D::operator[](const kit::uuid id)
{
    for (body2D &body : m_bodies)
        if (body.id == id)
            return &body;
    return nullptr;
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

std::vector<body2D::const_ptr> body_manager2D::operator[](const aabb2D &aabb) const
{
    std::vector<body2D::const_ptr> in_area;
    in_area.reserve(m_bodies.size() / 2);

    for (const body2D &body : m_bodies)
        if (geo::intersects(body.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_bodies, body.index);
    return in_area;
}
std::vector<body2D::ptr> body_manager2D::operator[](const aabb2D &aabb)
{
    std::vector<body2D::ptr> in_area;
    in_area.reserve(m_bodies.size() / 2);
    for (const body2D &body : m_bodies)
        if (geo::intersects(body.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_bodies, body.index);
    return in_area;
}

const body2D *body_manager2D::operator[](const glm::vec2 &point) const
{
    const aabb2D aabb = point;
    for (const body2D &body : m_bodies)
        if (geo::intersects(body.shape().bounding_box(), aabb))
            return &body;
    return nullptr;
}
body2D *body_manager2D::operator[](const glm::vec2 &point)
{
    const aabb2D aabb = point;
    for (body2D &body : m_bodies)
        if (geo::intersects(body.shape().bounding_box(), aabb))
            return &body;
    return nullptr;
}

void body_manager2D::process_addition(body2D &body)
{
    body.index = m_bodies.size() - 1;
    body.world = &world;

    const kit::transform2D<float> &transform = body.transform();

    rk::state<float> &state = world.integrator.state;
    state.append({transform.position.x, transform.position.y, transform.rotation, body.velocity.x, body.velocity.y,
                  body.angular_velocity});
    body.retrieve_data_from_state_variables(state.vars());

    KIT_INFO("Added body with index {0} and id {1}.", body.index, (std::uint64_t)body.id)
#ifdef DEBUG
    for (std::size_t i = 0; i < m_bodies.size() - 1; i++)
        KIT_ASSERT_CRITICAL(m_bodies[i].id != body.id, "Body with index {0} has the same id as body with index {1}", i,
                            body.index)
#endif
    world.events.on_body_addition(body);
}

bool body_manager2D::remove(const std::size_t index)
{
    if (index >= m_bodies.size())
    {
        KIT_WARN("Body index exceeds array bounds. Aborting... - index: {0}, size: {1}", index, m_bodies.size())
        return false;
    }
    KIT_INFO("Removing body with id {0}", m_bodies[index].id)

    world.events.on_early_body_removal(m_bodies[index]);
    rk::state<float> &state = world.integrator.state;
    if (index != m_bodies.size() - 1)
    {
        m_bodies[index] = m_bodies.back();
        m_bodies[index].index = index;
    }
    m_bodies.pop_back();

    for (std::size_t i = 0; i < 6; i++)
        state[6 * index + i] = state[state.size() - 6 + i];
    state.resize(6 * m_bodies.size());

    world.validate();
    world.events.on_late_body_removal(index);
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
        it->world = &world;
        it->index = index;
    }
}

void body_manager2D::send_data_to_state(rk::state<float> &state)
{
    KIT_PERF_FUNCTION()
    for (const body2D &body : m_bodies)
    {
        const std::size_t index = 6 * body.index;
        const glm::vec2 &position = body.position();

        state[index] = position.x;
        state[index + 1] = position.y;
        state[index + 2] = body.rotation();
        state[index + 3] = body.velocity.x;
        state[index + 4] = body.velocity.y;
        state[index + 5] = body.angular_velocity;
    }
}

void body_manager2D::retrieve_data_from_state_variables(const std::vector<float> &vars_buffer)
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
        body.retrieve_data_from_state_variables(vars_buffer);
}

void body_manager2D::prepare_constraint_velocities()
{
    for (body2D &body : m_bodies)
    {
        body.constraint_velocity = body.velocity + body.inv_mass() * body.force() * world.integrator.ts.value;
        body.constraint_angular_velocity =
            body.angular_velocity + body.inv_inertia() * body.torque() * world.integrator.ts.value;
    }
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