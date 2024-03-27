#include "ppx/internal/pch.hpp"
#include "ppx/body/body_manager2D.hpp"
#include "ppx/world2D.hpp"
#include "geo/algorithm/intersection.hpp"

namespace ppx
{
body2D *body_manager2D::add(const body2D::specs &spc)
{
    body2D *body = m_allocator.create(world, spc);
    body->begin_density_update();
    for (const auto &collider_spc : spc.props.colliders)
        body->add(collider_spc);
    body->end_density_update();

    const kit::transform2D<float> &centroid = body->centroid_transform();
    const glm::vec2 &velocity = body->velocity();

    rk::state<float> &state = world.integrator.state;
    state.append({centroid.position.x, centroid.position.y, centroid.rotation, velocity.x, velocity.y,
                  body->angular_velocity()});

    m_elements.push_back(body);
    events.on_addition(body);
    KIT_INFO("Added body with index {0}.", m_elements.size() - 1)
    return body;
}

void body_manager2D::apply_impulse_and_persistent_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D *body : m_elements)
    {
        body->apply_simulation_force(body->impulse_force + body->persistent_force);
        body->apply_simulation_torque(body->impulse_torque + body->persistent_torque);
    }
}

void body_manager2D::reset_impulse_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D *body : m_elements)
    {
        body->impulse_force = glm::vec2(0.f);
        body->impulse_torque = 0.f;
    }
}
void body_manager2D::reset_simulation_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D *body : m_elements)
        body->reset_simulation_forces();
}

template <typename Body, typename Collider, typename C>
static std::vector<Body *> in_area(C &elements, const aabb2D &aabb)
{
    std::vector<Body *> in_area;
    in_area.reserve(8);

    for (Body *body : elements)
        if (body->empty() && geo::intersects(aabb, body->centroid()))
        {
            in_area.push_back(body);
            break;
        }
        else
        {
            bool intersects = true;
            for (Collider *collider : *body)
                if (!geo::intersects(collider->bounding_box(), aabb))
                {
                    intersects = false;
                    break;
                }
            if (intersects)
                in_area.push_back(body);
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
    for (Body *body : elements)
        for (Collider *collider : *body)
            if (geo::intersects(collider->bounding_box(), point))
                return body;
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

    body2D *body = m_elements[index];
    KIT_INFO("Removing body with index {0}.", index)

    events.on_removal(*body);
    body->clear();

    rk::state<float> &state = world.integrator.state;
    if (index != m_elements.size())
    {
        for (std::size_t i = 0; i < 6; i++)
            state[6 * index + i] = state[state.size() - 6 + i];
        m_elements[index] = m_elements.back();
        m_elements[index]->index = index;
    }
    m_elements.pop_back();
    state.resize(6 * m_elements.size());

    world.on_body_removal_validation(body);
    m_allocator.destroy(body);
    return true;
}

void body_manager2D::send_data_to_state(rk::state<float> &state)
{
    KIT_PERF_FUNCTION()
    for (body2D *body : m_elements)
    {
        const std::size_t index = 6 * body->index;
        const glm::vec2 &centroid = body->centroid();
        glm::vec2 &velocity = body->velocity();
        if (body->is_static())
            velocity = glm::vec2(0.f);

        state[index] = centroid.x;
        state[index + 1] = centroid.y;
        state[index + 2] = body->rotation();
        state[index + 3] = velocity.x;
        state[index + 4] = velocity.y;
        state[index + 5] = body->angular_velocity();
    }
}

void body_manager2D::retrieve_data_from_state_variables(const std::vector<float> &vars_buffer)
{
    KIT_PERF_FUNCTION()
    for (body2D *body : m_elements)
        body->retrieve_data_from_state_variables(vars_buffer);
}

void body_manager2D::prepare_constraint_states()
{
    for (body2D *body : m_elements)
        body->ctr_state = body->m_state;
}

} // namespace ppx