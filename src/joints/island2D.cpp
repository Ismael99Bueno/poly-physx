#include "ppx/internal/pch.hpp"
#include "ppx/joints/island2D.hpp"
#include "ppx/body/body_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void island2D::add_body(body2D *body)
{
    KIT_ASSERT_ERROR(body->is_dynamic(), "Body must be dynamic")
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
            return;
        }
    KIT_WARN("Body not found in island");
}

void island2D::merge(island2D &island)
{
    for (body2D *body : island.m_bodies)
    {
        KIT_ASSERT_ERROR(body->is_dynamic(), "Body must be dynamic")
        m_bodies.push_back(body);
        body->meta.island = this;
    }
    m_actuators.insert(m_actuators.end(), island.m_actuators.begin(), island.m_actuators.end());
    m_constraints.insert(m_constraints.end(), island.m_constraints.begin(), island.m_constraints.end());
    island.merged = true;
    awake();
}

void island2D::prepare_constraint_states()
{
    body_manager2D::prepare_constraint_states(m_bodies, world.rk_substep_timestep(), world.semi_implicit_integration);
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

    prepare_constraint_states();
    const std::size_t viters = world.constraints.velocity_iterations;
    const std::size_t piters = world.constraints.position_iterations;

    for (constraint2D *constraint : m_constraints)
        if (constraint->enabled)
            constraint->startup();

    for (std::size_t i = 0; i < viters; i++)
        for (constraint2D *constraint : m_constraints)
            if (constraint->enabled)
                constraint->solve_velocities();

    bool solved = true;
    for (std::size_t i = 0; i < piters; i++)
    {
        solved = true;
        for (constraint2D *constraint : m_constraints)
            if (constraint->enabled)
                solved &= constraint->solve_positions();
        if (solved)
            break;
    }
    float energy = 0.f;
    for (body2D *body : m_bodies)
        energy += body->kinetic_energy();
    energy /= m_bodies.size();

    if (energy < world.islands.sleep_energy_threshold)
    {
        m_time_still += world.rk_substep_timestep();
        m_asleep = solved && m_time_still >= world.islands.sleep_time_threshold;
    }
    else
        m_time_still = 0.f;
}

bool island2D::empty() const
{
    return m_bodies.empty() && m_actuators.empty() && m_constraints.empty();
}
std::size_t island2D::size() const
{
    return m_bodies.size() + m_actuators.size() + m_constraints.size();
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
