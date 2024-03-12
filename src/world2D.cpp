#include "ppx/internal/pch.hpp"
#include "ppx/world2D.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/joints/spring2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#ifdef KIT_USE_YAML_CPP
#include "ppx/serialization/serialization.hpp"
#endif

#include <cstring>
#ifdef DEBUG
#include "kit/missing/fenv.h"
#endif

namespace ppx
{
void world2D::add(const specs::contraption2D &contraption)
{
    for (const body2D::specs &body : contraption.bodies)
        bodies.add(body);
    for (const spring2D::specs &spring : contraption.springs)
        joints.add<spring2D>(spring);
    for (const distance_joint2D::specs &joint : contraption.distance_joints)
        joints.add<distance_joint2D>(joint);
}

bool world2D::step()
{
    pre_step_preparation();
    const bool valid = integrator.raw_forward(*this);
    post_step_setup();
    return valid;
}

void world2D::pre_step_preparation()
{
#ifdef DEBUG
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

    m_timestep_ratio = kit::approaches_zero(integrator.ts.value) ? 1.f : m_previous_timestep / integrator.ts.value;
    collisions.detection()->clear_cached_collisions();

    bodies.send_data_to_state(integrator.state);
}
void world2D::post_step_setup()
{
    bodies.reset_impulse_forces();
    bodies.retrieve_data_from_state_variables(integrator.state.vars());
    m_previous_timestep = integrator.ts.value;

#ifdef DEBUG
    fedisableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
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

        const glm::vec2 &accel = body.force() * body.props().dynamic.inv_mass;
        const float angaccel = body.torque() * body.props().dynamic.inv_inertia;

        const glm::vec2 velocity =
            semi_implicit_integration ? body.velocity + accel * integrator.ts.value : body.velocity;
        const float angular_velocity =
            semi_implicit_integration ? body.angular_velocity + angaccel * integrator.ts.value : body.angular_velocity;

        state_derivative[index] = velocity.x;
        state_derivative[index + 1] = velocity.y;
        state_derivative[index + 2] = angular_velocity;

        state_derivative[index + 3] = accel.x;
        state_derivative[index + 4] = accel.y;
        state_derivative[index + 5] = angaccel;
    }
    return state_derivative;
}

void world2D::validate()
{
    bodies.validate();
    colliders.validate();
    joints.constraint_based.validate();
    joints.non_constraint_based.validate();
    behaviours.validate();
}

void world2D::add_builtin_joint_managers()
{
    joints.add_manager<spring2D>("Springs");
    joints.add_manager<distance_joint2D>("Distance joints");
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
    for (const spring2D &sp : *joints.manager<spring2D>())
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

    bodies.apply_impulse_and_persistent_forces();
    behaviours.apply_forces();
    joints.non_constraint_based.solve();

    if (collisions.enabled)
        collisions.solve();

    bodies.prepare_constraint_velocities();
    joints.constraint_based.solve();
    return create_state_derivative();
}
} // namespace ppx