#include "ppx/internal/pch.hpp"
#include "ppx/body/body_manager2D.hpp"
#include "ppx/world2D.hpp"
#include "geo/algorithm/intersection2D.hpp"

namespace ppx
{
body2D *body_manager2D::add(const body2D::specs &spc)
{
    body2D *body = m_allocator.create(world, spc);
    body->begin_density_update();
    for (const auto &collider_spc : spc.props.colliders)
        body->add(collider_spc);
    body->end_density_update();

    rk::state<float> &state = world.integrator.state;
    state.append({body->centroid().x, body->centroid().y, body->rotation(), body->velocity().x, body->velocity().y,
                  body->angular_velocity()});

    m_elements.push_back(body);
    events.on_addition(body);
    KIT_INFO("Added body with index {0}.", m_elements.size() - 1)
    return body;
}

void body_manager2D::apply_instant_and_persistent_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D *body : m_elements)
    {
        body->apply_simulation_force(body->instant_force + body->persistent_force);
        body->apply_simulation_torque(body->instant_torque + body->persistent_torque);
    }
}

void body_manager2D::reset_instant_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D *body : m_elements)
    {
        body->instant_force = glm::vec2(0.f);
        body->instant_torque = 0.f;
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

    const glm::vec2 tr = aabb.max;
    const glm::vec2 bl = aabb.min;
    const glm::vec2 tl = {bl.x, tr.y};
    const glm::vec2 br = {tr.x, bl.y};
    const polygon aabb_poly{bl, br, tr, tl};
    for (Body *body : elements)
        if (body->empty() && geo::intersects(aabb, body->centroid()))
            in_area.push_back(body);
        else
        {
            bool intersects = true;
            for (Collider *collider : *body)
                if (!geo::intersects(collider->bounding_box(), aabb) || !geo::gjk(aabb_poly, collider->shape()))
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

template <typename Body, typename Collider, typename C>
static std::vector<Body *> at_point(C &elements, const glm::vec2 &point)
{
    std::vector<Body *> at_point;
    for (Body *body : elements)
        for (Collider *collider : *body)
            if (geo::intersects(collider->bounding_box(), point) && collider->shape().contains_point(point))
            {
                at_point.push_back(body);
                break;
            }
    return at_point;
}

std::vector<const body2D *> body_manager2D::operator[](const glm::vec2 &point) const
{
    return at_point<const body2D, const collider2D>(m_elements, point);
}
std::vector<body2D *> body_manager2D::operator[](const glm::vec2 &point)
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
        float &angular_velocity = body->angular_velocity();
        if (body->is_static())
        {
            velocity = glm::vec2(0.f);
            angular_velocity = 0.f;
        }

        state[index] = centroid.x;
        state[index + 1] = centroid.y;
        state[index + 2] = body->rotation();
        state[index + 3] = velocity.x;
        state[index + 4] = velocity.y;
        state[index + 5] = angular_velocity;
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
    {
        body->proxy.ctr_state = body->m_state;
        if (world.semi_implicit_integration)
        {
            body->proxy.ctr_state.velocity +=
                body->props().dynamic.inv_mass * body->force() * world.rk_substep_timestep();
            body->proxy.ctr_state.angular_velocity +=
                body->props().dynamic.inv_inertia * body->torque() * world.rk_substep_timestep();
        }
    }
}

} // namespace ppx