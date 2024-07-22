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
world2D::world2D(const specs::world2D &spc)
    : integrator(spc.integrator.tableau, spc.integrator.timestep), bodies(*this), colliders(*this), joints(*this),
      behaviours(*this), collisions(*this), islands(*this)
{
    bodies.params = spc.bodies;
    colliders.params = spc.colliders;
    joints.constraints.params = spc.joints.constraints;
    collisions.broad()->params = spc.collision.broad;
    collisions.contact_manager()->params = spc.collision.contacts;
    islands.params = spc.islands;
}

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

bool world2D::step()
{
    if (islands.enabled() && bodies.all_asleep()) [[unlikely]]
        return true;

    m_step_count++;
    pre_step();
    const bool valid = integrator.raw_forward(*this);
    post_step();
    return valid;
}
std::uint32_t world2D::step_count() const
{
    return m_step_count;
}

void world2D::pre_step()
{
    KIT_PERF_SCOPE("ppx::world2D::pre_step")
#if defined(DEBUG) && !defined(_MSC_VER) // little fix, seems feenableexcept does not exist on windows
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
    bodies.gather_and_load_states(integrator.state);

    // surprisingly, i can get away with computing collisions once per step no matter the rk order
    if (collisions.enabled())
        collisions.detect_and_create_contacts();
    if (islands.enabled())
        islands.try_split();

    KIT_ASSERT_ERROR(collisions.contact_manager()->checksum(bodies), "Contacts checksum failed")
    KIT_ASSERT_ERROR(bodies.checksum(), "Bodies checksum failed")
    KIT_ASSERT_ERROR(joints.checksum(), "Joints checksum failed")
    m_rk_substep_index = 0;
}

std::vector<float> world2D::operator()(const float time, const float timestep, const std::vector<float> &posvels)
{
    KIT_PERF_SCOPE("ppx::world2D::ODE")
    KIT_ASSERT_CRITICAL(posvels.size() == 6 * bodies.size(),
                        "Positions and velocities vector size must be exactly 6 times greater than the body array size "
                        "- posvels: {0}, body array: {1}",
                        posvels.size(), bodies.size())
    m_rk_timestep = timestep;
    if (m_rk_substep_index != 0)
        bodies.update_states(posvels);

    std::vector<state2D> &states = bodies.mutable_states();
    behaviours.load_forces(states);

    const bool islands_enabled = islands.enabled();
    if (islands_enabled)
    {
        islands.remove_invalid_and_gather_awake();
        islands.solve_actuators(states);

        bodies.integrate_velocities(timestep);
        islands.solve_velocity_constraints(states);
    }
    else
    {
        joints.actuators.solve(states);

        bodies.integrate_velocities(timestep);
        joints.constraints.solve_velocities(states);
    }

    bodies.integrate_positions(timestep);
    if (islands_enabled)
        islands.solve_position_constraints(states);
    else
        joints.constraints.solve_positions(states);

    m_rk_substep_index++;
    return bodies.load_velocities_and_forces();
}

void world2D::post_step()
{
    KIT_PERF_SCOPE("ppx::world2D::post_step")
    if (!bodies.retrieve_data_from_states(integrator.state.vars()))
        colliders.update_bounding_boxes();

    KIT_ASSERT_ERROR(!islands.enabled() || islands.checksum(), "Island checkusm failed")
#if defined(DEBUG) && !defined(_MSC_VER)
    fedisableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
}

std::uint32_t world2D::hertz() const
{
    if (kit::approaches_zero(integrator.ts.value))
        return 0;
    return (std::uint32_t)(1.f / integrator.ts.value);
}

float world2D::rk_timestep() const
{
    return m_rk_timestep;
}
float world2D::substep_timestep() const
{
    return integrator.ts.value / integrator.tableau().stages;
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
        if (bhv->enabled())
            pot += bhv->potential_energy();
    for (const spring_joint2D *sp : *joints.manager<spring_joint2D>())
        pot += sp->potential_energy();
    return pot;
}
float world2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

} // namespace ppx