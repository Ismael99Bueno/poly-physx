#include "ppx/internal/pch.hpp"
#include "ppx/body/body_manager2D.hpp"
#include "ppx/world2D.hpp"
#include "geo/algorithm/intersection2D.hpp"
#include "kit/multithreading/mt_for_each.hpp"

namespace ppx
{
body2D *body_manager2D::add(const body2D::specs &spc)
{
    KIT_PERF_SCOPE("ppx::body_manager2D::add")
    body2D *body = allocator<body2D>::create(world, spc);
    m_elements.push_back(body);

    body->begin_density_update();
    for (const auto &collider_spc : spc.props.colliders)
        body->add(collider_spc);
    body->end_density_update();

    if (world.islands.enabled() && body->is_dynamic())
    {
        island2D *island = world.islands.create_and_add();
        island->add_body(body);
    }

    events.on_addition(body);
    KIT_INFO("Added body with index {0}.", m_elements.size() - 1)
    return body;
}

std::vector<float> body_manager2D::load_velocities_and_forces() const
{
    KIT_PERF_SCOPE("ppx::body_manager2D::load_velocities_and_forces")
    std::vector<float> velaccels(6 * m_elements.size());

    for (std::size_t i = 0; i < m_elements.size(); i++)
    {
        const state2D &state = m_states[i];
        const std::size_t index = 6 * i;

        const glm::vec2 accel = state.substep_force * state.inv_mass();
        const float angaccel = state.substep_torque * state.inv_inertia();

        velaccels[index] = state.velocity.x;
        velaccels[index + 1] = state.velocity.y;
        velaccels[index + 2] = state.angular_velocity;
        velaccels[index + 3] = accel.x;
        velaccels[index + 4] = accel.y;
        velaccels[index + 5] = angaccel;
    }
    return velaccels;
}

template <typename Body, typename Collider, typename C>
static std::vector<Body *> in_area(C &elements, const aabb2D &aabb)
{
    std::vector<Body *> in_area;
    if (kit::approaches_zero(aabb.area()))
        return in_area;
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
                if (!geo::intersects(collider->tight_bbox(), aabb) || !geo::gjk(aabb_poly, collider->shape()))
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
            if (geo::intersects(collider->tight_bbox(), point) && collider->shape().contains_point(point))
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

bool body_manager2D::all_asleep() const
{
    for (const body2D *body : m_elements)
        if (!body->asleep())
            return false;
    return true;
}

const std::vector<state2D> &body_manager2D::states() const
{
    return m_states;
}

bool body_manager2D::checksum() const
{
    const std::size_t colliders = world.colliders.size();
    std::size_t collider_count1 = 0;
    std::size_t collider_count2 = 0;

    for (const body2D *body : m_elements)
    {
        if (!body->checksum())
            return false;
        collider_count1 += body->size();
        for (const collider2D *collider : *body)
        {
            (void)collider;
            collider_count2++;
        }
    }
    KIT_ASSERT_ERROR(collider_count1 == collider_count2 && collider_count1 == colliders, "Collider count mismatch")
    return collider_count1 == collider_count2 && collider_count1 == colliders;
}

bool body_manager2D::remove(const std::size_t index)
{
    if (index >= m_elements.size())
        return false;

    body2D *body = m_elements[index];
    KIT_INFO("Removing body with index {0}.", index)

    events.on_removal(*body);
    body->clear();

    if (index != m_elements.size())
    {
        m_elements[index] = m_elements.back();
        m_elements[index]->meta.index = index;
    }
    m_elements.pop_back();

    KIT_ASSERT_ERROR(body->meta.island || !body->is_dynamic() || !world.islands.enabled(),
                     "Body is not in an island when it should be!")

    world.on_body_removal_validation(body);
    if (body->meta.island)
        body->meta.island->remove_body(body);
    allocator<body2D>::destroy(body);
    return true;
}

void body_manager2D::gather_and_load_states(rk::state<float> &rkstate)
{
    KIT_PERF_SCOPE("ppx::body_manager2D::gather_and_load_states")
    m_states.resize(m_elements.size());
    rkstate.resize(6 * m_elements.size());
    for (std::size_t i = 0; i < m_elements.size(); i++)
    {
        m_states[i] = m_elements[i]->state();
        m_states[i].substep_force = glm::vec2(0.f);
        m_states[i].substep_torque = 0.f;

        const state2D &state = m_states[i];
        const std::size_t index = 6 * i;

        rkstate[index] = state.centroid.position.x;
        rkstate[index + 1] = state.centroid.position.y;
        rkstate[index + 2] = state.centroid.rotation;
        rkstate[index + 3] = state.velocity.x;
        rkstate[index + 4] = state.velocity.y;
        rkstate[index + 5] = state.angular_velocity;
    }
}

void body_manager2D::update_states(const std::vector<float> &posvels)
{
    KIT_PERF_SCOPE("ppx::body_manager2D::update_states")
    for (std::size_t i = 0; i < m_elements.size(); i++)
    {
        state2D &state = m_states[i];

        state.substep_force = glm::vec2(0.f);
        state.substep_torque = 0.f;

        const std::size_t index = 6 * i;

        state.centroid.position.x = posvels[index];
        state.centroid.position.y = posvels[index + 1];
        state.centroid.rotation = posvels[index + 2];
        state.velocity.x = posvels[index + 3];
        state.velocity.y = posvels[index + 4];
        state.angular_velocity = posvels[index + 5];
    }
}

void body_manager2D::integrate_velocities(const float ts)
{
    KIT_PERF_SCOPE("ppx::body_manager2D::integrate_velocities")
    for (std::size_t i = 0; i < m_elements.size(); i++)
    {
        const body2D *body = m_elements[i];
        state2D &state = m_states[i];

        state.substep_force += body->instant_force() + body->persistent_force();
        state.substep_torque += body->instant_torque() + body->persistent_torque();

        state.velocity += state.substep_force * state.inv_mass() * ts;
        state.angular_velocity += state.substep_torque * state.inv_inertia() * ts;
    }
}

void body_manager2D::integrate_positions(const float ts)
{
    KIT_PERF_SCOPE("ppx::body_manager2D::integrate_positions")
    for (state2D &state : m_states)
    {
        state.centroid.position += state.velocity * ts;
        state.centroid.rotation += state.angular_velocity * ts;
    }
}

// id even dare to say that maybe separate this into a call to update states may be better
bool body_manager2D::retrieve_data_from_states(const std::vector<float> &posvels)
{
    KIT_PERF_SCOPE("ppx::body_manager2D::retrieve_data_from_states")
    const bool mt = params.multithreading;
    const auto lambda = [this, mt, &posvels](body2D *body) {
        body->m_instant_force = glm::vec2(0.f);
        body->m_instant_torque = 0.f;

        const std::size_t i = body->meta.index;
        state2D &state = m_states[i];
        state.substep_force = glm::vec2(0.f);
        state.substep_torque = 0.f;

        if (body->asleep()) [[unlikely]]
            return;

        const std::size_t index = 6 * i;
        state.centroid.position.x = posvels[index];
        state.centroid.position.y = posvels[index + 1];
        state.centroid.rotation = posvels[index + 2];
        state.velocity.x = posvels[index + 3];
        state.velocity.y = posvels[index + 4];
        state.angular_velocity = posvels[index + 5];

        body->retrieve_data_from_state(state, !mt);
    };

    const auto pool = world.thread_pool;
    if (mt && pool)
        kit::mt::for_each(*pool, m_elements.begin(), m_elements.end(), lambda, pool->thread_count());
    else
        for (body2D *body : m_elements)
            lambda(body);

    return !mt;
}

std::vector<state2D> &body_manager2D::mutable_states()
{
    return m_states;
}

} // namespace ppx