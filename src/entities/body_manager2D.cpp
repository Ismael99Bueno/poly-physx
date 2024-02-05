#include "ppx/internal/pch.hpp"
#include "ppx/entities/body_manager2D.hpp"
#include "ppx/world2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
body2D &body_manager2D::add(const body2D::specs &spc)
{
    body2D &body = m_elements.emplace_back(world, spc);
    body.index = m_elements.size() - 1;

    const kit::transform2D<float> &transform = body.transform();

    rk::state<float> &state = world.integrator.state;
    state.append({transform.position.x, transform.position.y, transform.rotation, body.velocity.x, body.velocity.y,
                  body.angular_velocity});
    body.retrieve_data_from_state_variables(state.vars());

    world.colliders.validate_parents();
    events.on_addition(body);
    KIT_INFO("Added body with index {0} and id {1}.", body.index, body.id)
    return body;
}

void body_manager2D::apply_impulse_and_persistent_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_elements)
    {
        body.apply_simulation_force(body.impulse_force + body.persistent_force);
        body.apply_simulation_torque(body.impulse_torque + body.persistent_torque);
    }
}

void body_manager2D::reset_impulse_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_elements)
    {
        body.impulse_force = glm::vec2(0.f);
        body.impulse_torque = 0.f;
    }
}
void body_manager2D::reset_simulation_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_elements)
        body.reset_simulation_forces();
}

body2D::const_ptr body_manager2D::ptr(const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    return {&m_elements, index};
}
body2D::ptr body_manager2D::ptr(const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_elements.size())
    return {&m_elements, index};
}

template <typename Body, typename Collider, typename C>
static std::vector<Body *> in_area(C &elements, const aabb2D &aabb)
{
    std::vector<Body *> in_area;
    in_area.reserve(elements.size() / 2);

    for (Body &body : elements)
        if (geo::intersects(aabb, body.centroid()))
        {
            in_area.push_back(&body);
            break;
        }
        else
            for (Collider &collider : body)
                if (geo::intersects(collider.bounding_box(), aabb))
                {
                    in_area.push_back(&body);
                    break;
                }
    return in_area;
}

std::vector<const body2D *> body_manager2D::operator[](const aabb2D &aabb) const
{
    return in_area<const body2D, const collider2D>(m_elements, aabb);
}
std::vector<body2D *> body_manager2D::operator[](const aabb2D &aabb)
{
    return in_area<body2D, collider2D>(m_elements, aabb);
}

template <typename Body, typename Collider, typename C> static Body *at_point(C &elements, const glm::vec2 &point)
{
    for (Body &body : elements)
        for (Collider &collider : body)
            if (geo::intersects(collider.bounding_box(), point))
                return &body;
    return nullptr;
}

const body2D *body_manager2D::operator[](const glm::vec2 &point) const
{
    return at_point<const body2D, const collider2D>(m_elements, point);
}
body2D *body_manager2D::operator[](const glm::vec2 &point)
{
    return at_point<body2D, collider2D>(m_elements, point);
}

bool body_manager2D::remove(const std::size_t index)
{
    if (index >= m_elements.size())
        return false;

    KIT_INFO("Removing body with id {0}", m_elements[index].id)

    events.on_early_removal(m_elements[index]);
    m_elements[index].clear();
    if (index != m_elements.size() - 1)
    {
        m_elements[index] = m_elements.back();
        m_elements[index].index = index;
    }
    m_elements.pop_back();

    rk::state<float> &state = world.integrator.state;
    for (std::size_t i = 0; i < 6; i++)
        state[6 * index + i] = state[state.size() - 6 + i];
    state.resize(6 * m_elements.size());

    validate();
    world.colliders.validate_parents();
    world.constraints.validate();
    world.behaviours.validate();
    world.springs.validate();
    events.on_late_removal(index);
    return true;
}

void body_manager2D::validate()
{
    std::size_t index = 0;
    for (auto it = m_elements.begin(); it != m_elements.end(); index++, ++it)
        it->index = index;
}

void body_manager2D::send_data_to_state(rk::state<float> &state)
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_elements)
    {
        const std::size_t index = 6 * body.index;
        const glm::vec2 &position = body.position();
        if (body.is_dynamic())
            body.velocity = glm::vec2(0.f);

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
    for (body2D &body : m_elements)
        body.retrieve_data_from_state_variables(vars_buffer);
}

void body_manager2D::prepare_constraint_velocities()
{
    for (body2D &body : m_elements)
    {
        body.ctr_proxy.velocity =
            body.velocity + body.props().dynamic.inv_mass * body.force() * world.integrator.ts.value;
        body.ctr_proxy.angular_velocity =
            body.angular_velocity + body.props().dynamic.inv_inertia * body.torque() * world.integrator.ts.value;
    }
}

} // namespace ppx