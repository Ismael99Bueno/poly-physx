#include "ppx/internal/pch.hpp"
#include "ppx/island/island2D.hpp"
#include "ppx/body/body_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void island2D::add_body(body2D *body)
{
    KIT_ASSERT_ERROR(body->is_dynamic(), "Body must be dynamic")
    KIT_ASSERT_ERROR(body->meta.island != this, "Body already in island")
    KIT_ASSERT_ERROR(std::find(m_bodies.begin(), m_bodies.end(), body) == m_bodies.end(),
                     "Body already in island, but not labeled as such")
    m_bodies.push_back(body);
    body->meta.island = this;
    awake();
}

void island2D::awake()
{
    m_asleep = false;
    m_time_still = 0.f;
}
bool island2D::asleep() const
{
    return m_asleep;
}

void island2D::remove_body(body2D *body)
{
    for (std::size_t i = 0; i < m_bodies.size(); i++)
        if (m_bodies[i] == body)
        {
            m_bodies.erase(m_bodies.begin() + i);
            body->meta.island = nullptr;
            awake();
            may_split = true;
            if (no_bodies())
                world.islands.remove(this);
            return;
        }
    KIT_WARN("Body not found in island");
}

void island2D::merge(island2D &island)
{
    for (body2D *body : island.m_bodies)
    {
        KIT_ASSERT_ERROR(body->is_dynamic(), "Body must be dynamic")
        KIT_ASSERT_ERROR(std::find(m_bodies.begin(), m_bodies.end(), body) == m_bodies.end(), "Body already in island")
        m_bodies.push_back(body);
        body->meta.island = this;
    }
    m_actuators.insert(m_actuators.end(), island.m_actuators.begin(), island.m_actuators.end());
    m_constraints.insert(m_constraints.end(), island.m_constraints.begin(), island.m_constraints.end());
    island.merged = true;
    awake();
}

const std::vector<body2D *> &island2D::bodies() const
{
    return m_bodies;
}
const std::vector<actuator2D *> &island2D::actuators() const
{
    return m_actuators;
}
const std::vector<constraint2D *> &island2D::constraints() const
{
    return m_constraints;
}

void island2D::solve()
{
    for (actuator2D *actuator : m_actuators)
        if (actuator->enabled)
            actuator->solve();

    for (body2D *body : m_bodies)
        body->prepare_constraint_states();
    const std::size_t viters = world.constraints.velocity_iterations;
    const std::size_t piters = world.constraints.position_iterations;

    for (constraint2D *constraint : m_constraints)
        if (constraint->enabled)
            constraint->startup();

    for (std::size_t i = 0; i < viters; i++)
        for (constraint2D *constraint : m_constraints)
            if (constraint->enabled)
                constraint->solve_velocities();

    m_solved_positions = true;
    for (std::size_t i = 0; i < piters; i++)
    {
        m_solved_positions = true;
        for (constraint2D *constraint : m_constraints)
            if (constraint->enabled)
                m_solved_positions &= constraint->solve_positions();
        if (m_solved_positions)
            break;
    }
    if (world.rk_subset_index() != 0)
        return;

    m_energy = 0.f;
    for (body2D *body : m_bodies)
        m_energy += body->kinetic_energy();
    m_energy /= m_bodies.size();

    if (m_energy < world.islands.sleep_energy_threshold)
    {
        m_time_still += world.rk_substep_timestep();
        m_asleep = m_solved_positions && m_time_still >= world.islands.sleep_time_threshold;
    }
    else
        m_time_still = 0.f;
}

bool island2D::is_void() const
{
    return no_bodies() && no_joints();
}
bool island2D::no_bodies() const
{
    return m_bodies.empty();
}
bool island2D::no_joints() const
{
    return m_actuators.empty() && m_constraints.empty();
}

std::size_t island2D::size() const
{
    return m_bodies.size() + m_actuators.size() + m_constraints.size();
}

float island2D::time_still() const
{
    return m_time_still;
}
float island2D::energy() const
{
    return m_energy;
}
bool island2D::solved_positions() const
{
    return m_solved_positions;
}

island2D *island2D::handle_island_merge_encounter(island2D *island1, island2D *island2)
{
    if (island1 == island2)
        return island1;
    if (island1->size() > island2->size())
    {
        island1->merge(*island2);
        return island1;
    }
    island2->merge(*island1);
    return island2;
}
} // namespace ppx
