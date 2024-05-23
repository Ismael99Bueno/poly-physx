#include "ppx/internal/pch.hpp"
#include "ppx/world2D.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/joints/spring_joint2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include "ppx/joints/weld_joint2D.hpp"
#include "ppx/joints/rotor_joint2D.hpp"
#include "ppx/joints/motor_joint2D.hpp"
#include "ppx/joints/ball_joint2D.hpp"
#include "ppx/joints/prismatic_joint2D.hpp"
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
    for (const spring_joint2D::specs &spring : contraption.springs)
        joints.add<spring_joint2D>(spring);
    for (const distance_joint2D::specs &joint : contraption.distance_joints)
        joints.add<distance_joint2D>(joint);
    for (const revolute_joint2D::specs &joint : contraption.revolute_joints)
        joints.add<revolute_joint2D>(joint);
    for (const weld_joint2D::specs &joint : contraption.weld_joints)
        joints.add<weld_joint2D>(joint);
    for (const rotor_joint2D::specs &joint : contraption.rotor_joints)
        joints.add<rotor_joint2D>(joint);
    for (const motor_joint2D::specs &joint : contraption.motor_joints)
        joints.add<motor_joint2D>(joint);
    for (const ball_joint2D::specs &joint : contraption.ball_joints)
        joints.add<ball_joint2D>(joint);
    for (const prismatic_joint2D::specs &joint : contraption.prismatic_joints)
        joints.add<prismatic_joint2D>(joint);
}

world2D::island_settings::island_settings(joint_repository2D &jr) : m_joints(jr)
{
}

bool world2D::island_settings::enabled() const
{
    return m_joints.islands_enabled();
}
void world2D::island_settings::enabled(bool enable)
{
    m_joints.islands_enabled(enable);
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
    KIT_ASSERT_ERROR(!joints.islands_enabled() || joints.island_checksum(), "Checksum failed")
    collisions.detection()->flag_new_step();
    bodies.send_data_to_state(integrator.state);
    if (joints.islands_enabled())
        joints.try_split_islands(1);
}
void world2D::post_step_setup()
{
    bodies.reset_instant_forces();
    bodies.retrieve_data_from_state_variables(integrator.state.vars());
    m_previous_timestep = integrator.ts.value;

    KIT_ASSERT_ERROR(!joints.islands_enabled() || joints.island_checksum(), "Checksum failed")
#ifdef DEBUG
    fedisableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
}

float world2D::rk_substep_timestep() const
{
    return m_rk_substep_timestep;
}

std::vector<float> world2D::create_state_derivative() const
{
    KIT_PERF_FUNCTION()
    std::vector<float> state_derivative(6 * bodies.size(), 0.f);

    for (const body2D *body : bodies)
    {
        const std::size_t index = 6 * body->index;

        const glm::vec2 &accel = body->force() * body->props().dynamic.inv_mass;
        const float angaccel = body->torque() * body->props().dynamic.inv_inertia;

        const glm::vec2 velocity =
            semi_implicit_integration ? body->velocity() + accel * m_rk_substep_timestep : body->velocity();
        const float angular_velocity = semi_implicit_integration
                                           ? body->angular_velocity() + angaccel * m_rk_substep_timestep
                                           : body->angular_velocity();

        state_derivative[index] = velocity.x;
        state_derivative[index + 1] = velocity.y;
        state_derivative[index + 2] = angular_velocity;

        state_derivative[index + 3] = accel.x;
        state_derivative[index + 4] = accel.y;
        state_derivative[index + 5] = angaccel;
    }
    return state_derivative;
}

void world2D::on_body_removal_validation(body2D *body)
{
    joints.constraints.on_body_removal_validation(body);
    joints.actuators.on_body_removal_validation(body);
    behaviours.on_body_removal_validation(body);
}

void world2D::add_builtin_joint_managers()
{
    joints.add_manager<spring_joint2D>("Spring joints");
    joints.add_manager<distance_joint2D>("Distance joints");
    joints.add_manager<revolute_joint2D>("Revolute joints");
    joints.add_manager<weld_joint2D>("Weld joints");
    joints.add_manager<rotor_joint2D>("Rotor joints");
    joints.add_manager<motor_joint2D>("Motor joints");
    joints.add_manager<ball_joint2D>("Ball joints");
    joints.add_manager<prismatic_joint2D>("Prismatic joints");
}

float world2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const body2D *body : bodies)
        ke += body->kinetic_energy();
    return ke;
}
float world2D::potential_energy() const
{
    float pot = 0.f;
    for (const auto &bhv : behaviours)
        if (bhv->enabled)
            pot += bhv->potential_energy();
    for (const spring_joint2D *sp : *joints.manager<spring_joint2D>())
        pot += sp->potential_energy();
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

    m_rk_substep_timestep = timestep;
    bodies.prepare_for_next_substep(vars);

    behaviours.apply_forces();
    if (collisions.enabled)
        collisions.detect_and_resolve();

    if (joints.islands_enabled())
        joints.solve_islands();
    else
    {
        joints.actuators.solve();
        bodies.prepare_constraint_states();
        joints.constraints.solve();
    }
    return create_state_derivative();
}
} // namespace ppx