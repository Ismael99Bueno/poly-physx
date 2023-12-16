#include "ppx/internal/pch.hpp"
#include "ppx/world2D.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/collision/solvers/collision_solver2D.hpp"
#include <cstring>
#ifdef DEBUG
#include "kit/missing/fenv.h"
#endif

namespace ppx
{
world2D::world2D(const rk::butcher_tableau &table)
    : integrator(table), bodies(*this), springs(*this), behaviours(*this), collisions(*this), constraints(*this)
{
}

bool world2D::raw_forward(const float timestep)
{
    pre_step_preparation(timestep);
    const bool valid = integrator.raw_forward(m_elapsed, timestep, *this);
    post_step_setup();
    return valid;
}
bool world2D::reiterative_forward(float &timestep, const std::uint8_t reiterations)
{
    pre_step_preparation(timestep);
    const bool valid = integrator.reiterative_forward(m_elapsed, timestep, *this, reiterations);
    post_step_setup();
    return valid;
}
bool world2D::embedded_forward(float &timestep)
{
    pre_step_preparation(timestep);
    const bool valid = integrator.embedded_forward(m_elapsed, timestep, *this);
    post_step_setup();
    return valid;
}

void world2D::pre_step_preparation(const float timestep)
{
#ifdef DEBUG
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

    m_timestep_ratio = kit::approaches_zero(timestep) ? 1.f : m_current_timestep / timestep;
    m_current_timestep = timestep;

    collisions.detection()->clear_cached_collisions();
    constraints.reset_delegated_collisions();

    bodies.send_data_to_state(integrator.state);
}
void world2D::post_step_setup()
{
    bodies.reset_added_forces();
    bodies.retrieve_data_from_state_variables(integrator.state.vars());
    if (collision_detection2D::build_contact_manifold_over_time)
        collisions.detection()->query_last_contact_points();

#ifdef DEBUG
    fedisableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
}

float world2D::current_timestep() const
{
    return m_current_timestep;
}
float world2D::timestep_ratio() const
{
    return m_timestep_ratio;
}

std::vector<float> world2D::create_state_derivative() const
{
    KIT_PERF_FUNCTION()
    std::vector<float> state_derivative(6 * bodies.size(), 0.f);

    for (const body2D &body : bodies)
    {
        const std::size_t index = 6 * body.index;
        const glm::vec2 &velocity = body.velocity();
        const float angular_velocity = body.angular_velocity();
        state_derivative[index] = velocity.x;
        state_derivative[index + 1] = velocity.y;
        state_derivative[index + 2] = angular_velocity;

        const glm::vec2 &accel = body.force() * body.inv_mass();
        const float angaccel = body.torque() * body.inv_inertia();
        state_derivative[index + 3] = accel.x;
        state_derivative[index + 4] = accel.y;
        state_derivative[index + 5] = angaccel;
    }
    return state_derivative;
}

void world2D::validate()
{
    bodies.validate();
    constraints.validate();
    behaviours.validate();
    springs.validate();
}

float world2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const body2D &body : bodies)
        ke += body.kinetic_energy();
    return ke;
}
float world2D::potential_energy() const
{
    float pot = 0.f;
    for (const auto &bhv : behaviours)
        if (bhv->enabled)
            pot += bhv->potential_energy();
    for (const spring2D &sp : springs)
        pot += sp.potential_energy();
    return pot;
}
float world2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

std::vector<float> world2D::operator()(const float time, const float timestep, const std::vector<float> &vars)
{
    KIT_PERF_FUNCTION()
    KIT_ASSERT_CRITICAL(
        vars.size() == 6 * bodies.size(),
        "State vector size must be exactly 6 times greater than the body array size - vars: {0}, body array: {1}",
        vars.size(), bodies.size())

    bodies.reset_simulation_forces();
    bodies.retrieve_data_from_state_variables(vars);

    bodies.apply_added_forces();
    behaviours.apply_forces();
    springs.apply_forces();

    if (collisions.enabled)
        collisions.solve();

    constraints.solve();
    return create_state_derivative();
}

float world2D::elapsed() const
{
    return m_elapsed;
}
#ifdef KIT_USE_YAML_CPP
YAML::Node world2D::serializer::encode(const world2D &world) const
{
    YAML::Node node;
    for (const body2D &body : world.bodies)
        node["Bodies"].push_back(body);

    for (const spring2D &sp : world.springs)
        node["Springs"].push_back(sp);
    for (const auto &ctr : world.constraints)
    {
        YAML::Node child;
        child[ctr->name] = *ctr;
        node["Constraints"].push_back(child);
    }

    for (const auto &bhv : world.behaviours)
        node["Behaviours"][bhv->id] = *bhv;

    node["Integrator"] = world.integrator;
    node["Elapsed"] = world.elapsed();
    return node;
}
bool world2D::serializer::decode(const YAML::Node &node, world2D &world) const
{
    if (!node.IsMap() || node.size() < 3)
        return false;

    world.bodies.clear();
    world.integrator = node["Integrator"].as<rk::integrator>();
    world.integrator.state.clear();

    if (node["Bodies"])
        for (const YAML::Node &n : node["Bodies"])
            world.bodies.add(n.as<body2D>());

    if (node["Springs"])
        for (const YAML::Node &n : node["Springs"])
        {
            spring2D sp;
            sp.world = &world;
            n.as<spring2D>(sp);
            world.springs.add(sp);
        }

    if (node["Constraints"])
        for (const YAML::Node &n : node["Constraints"])
            if (n["Distance"])
            {
                distance_joint2D dj;
                dj.world = &world;
                n["Distance"].as<distance_joint2D>(dj);
                world.constraints.add<distance_joint2D>(dj);
            }

    if (node["Behaviours"])
        for (auto it = node["Behaviours"].begin(); it != node["Behaviours"].end(); ++it)
        {
            const auto bhv = world.behaviours.from_name<behaviour2D>(it->first.as<std::string>());
            if (bhv)
                it->second.as<behaviour2D>(*bhv);
        }

    world.m_elapsed = node["Elapsed"].as<float>();
    return true;
}
#endif
} // namespace ppx