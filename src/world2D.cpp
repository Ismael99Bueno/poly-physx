#include "ppx/internal/pch.hpp"
#include "ppx/world2D.hpp"
#include "geo/intersection.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/joints/distance_joint2D.hpp"
#include "ppx/collision/quad_tree_detection2D.hpp"
#include "ppx/collision/spring_solver2D.hpp"
#include <cstring>

namespace ppx
{
world2D::world2D(const rk::butcher_tableau &table, const std::size_t allocations)
    : integrator(table), m_constraint_manager(*this, allocations)
{
    m_bodies.reserve(allocations);
    integrator.state.reserve(6 * allocations);

    set_collision_detection<quad_tree_detection2D>();
    set_collision_solver<spring_solver2D>();
}

void world2D::send_data_to_state_variables()
{
    KIT_PERF_FUNCTION()
    for (const body2D &body : m_bodies)
    {
        const std::size_t index = 6 * body.index;
        rk::state &st = integrator.state;

        const glm::vec2 &position = body.position();
        const glm::vec2 &velocity = body.velocity();
        const float rotation = body.rotation();
        const float angular_velocity = body.angular_velocity();

        st[index] = position.x;
        st[index + 1] = position.y;
        st[index + 2] = rotation;
        st[index + 3] = velocity.x;
        st[index + 4] = velocity.y;
        st[index + 5] = angular_velocity;
    }
}

void world2D::retrieve_data_from_state_variables(const std::vector<float> &vars_buffer)
{
    KIT_PERF_FUNCTION()
    for (std::size_t i = 0; i < m_bodies.size(); i++)
        m_bodies[i].retrieve_data_from_state_variables(vars_buffer);
}
void world2D::retrieve_data_from_state_variables()
{
    retrieve_data_from_state_variables(integrator.state.vars());
}

bool world2D::raw_forward(const float timestep)
{
    m_timestep_ratio = m_current_timestep / timestep;
    m_current_timestep = timestep;

    send_data_to_state_variables();
    const bool valid = integrator.raw_forward(m_elapsed, timestep, *this);
    reset_bodies_added_forces();

    retrieve_data_from_state_variables();
    m_collision_detection->clear_cached_collisions();
    return valid;
}
bool world2D::reiterative_forward(float &timestep, const std::uint8_t reiterations)
{
    m_timestep_ratio = m_current_timestep / timestep;
    m_current_timestep = timestep;

    send_data_to_state_variables();
    const bool valid = integrator.reiterative_forward(m_elapsed, timestep, *this, reiterations);
    reset_bodies_added_forces();

    retrieve_data_from_state_variables();
    m_collision_detection->clear_cached_collisions();
    return valid;
}
bool world2D::embedded_forward(float &timestep)
{
    m_timestep_ratio = m_current_timestep / timestep;
    m_current_timestep = timestep;

    send_data_to_state_variables();
    const bool valid = integrator.embedded_forward(m_elapsed, timestep, *this);
    reset_bodies_added_forces();

    retrieve_data_from_state_variables();
    m_collision_detection->clear_cached_collisions();
    return valid;
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
    std::vector<float> state_derivative(6 * m_bodies.size(), 0.f);

    for (const body2D &body : m_bodies)
    {
        const std::size_t index = 6 * body.index;
        const glm::vec2 &velocity = body.velocity();
        const float angular_velocity = body.angular_velocity();
        state_derivative[index] = velocity.x;
        state_derivative[index + 1] = velocity.y;
        state_derivative[index + 2] = angular_velocity;

        const glm::vec2 &accel = body.force() * body.effective_inverse_mass();
        const float angaccel = body.torque() * body.effective_inverse_inertia();
        state_derivative[index + 3] = accel.x;
        state_derivative[index + 4] = accel.y;
        state_derivative[index + 5] = angaccel;
    }
    return state_derivative;
}

void world2D::validate()
{
    m_constraint_manager.validate(events.on_constraint_removal);
    for (const auto &bhv : m_behaviours)
        bhv->validate();
    for (auto it = m_springs.begin(); it != m_springs.end();)
        if (!it->joint.valid())
        {
            events.on_early_spring_removal(*it);
            std::size_t index = it->index;
            it = m_springs.erase(it);
            events.on_late_spring_removal(std::move(index));
        }
        else
            ++it;
}

void world2D::apply_world_behaviours_and_springs()
{
    KIT_PERF_FUNCTION()
    for (const auto &bhv : m_behaviours)
        if (bhv->enabled)
            bhv->apply_force_to_bodies();
    for (spring2D &sp : m_springs)
        sp.apply_force_to_bodies();
}
void world2D::apply_added_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
    {
        body.apply_simulation_force(body.added_force() + body.persistent_force);
        body.apply_simulation_torque(body.added_torque() + body.persistent_torque);
    }
}

void world2D::reset_bodies_added_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
        body.reset_added_forces();
}
void world2D::reset_bodies_simulation_forces()
{
    KIT_PERF_FUNCTION()
    for (body2D &body : m_bodies)
        body.reset_simulation_forces();
}

body2D::ptr world2D::process_body_addition(body2D &body)
{
    body.index = m_bodies.size() - 1;
    const body2D::ptr e_ptr = {&m_bodies, m_bodies.size() - 1};

    const kit::transform2D &transform = body.transform();
    const glm::vec2 &velocity = body.velocity();

    rk::state &state = integrator.state;
    state.append({transform.position.x, transform.position.y, transform.rotation, velocity.x, velocity.y,
                  body.angular_velocity()});
    body.retrieve_data_from_state_variables(state.vars());

    KIT_INFO("Added body with index {0} and id {1}.", body.index, (std::uint64_t)body.id)
#ifdef DEBUG
    for (std::size_t i = 0; i < m_bodies.size() - 1; i++)
        KIT_ASSERT_CRITICAL(m_bodies[i].id != body.id, "Body with index {0} has the same id as body with index {1}", i,
                            body.index)
#endif
    events.on_body_addition(e_ptr);
    return e_ptr;
}

bool world2D::remove_body(std::size_t index)
{
    if (index >= m_bodies.size())
    {
        KIT_WARN("Body index exceeds array bounds. Aborting... - index: {0}, size: {1}", index, m_bodies.size())
        return false;
    }
    KIT_INFO("Removing body with index {0} and id {1}", index, m_bodies[index].id)

    events.on_early_body_removal(m_bodies[index]);
    rk::state &state = integrator.state;
    if (index != m_bodies.size() - 1)
    {
        m_bodies[index] = m_bodies.back();
        m_bodies[index].index = index;
    }
    m_bodies.pop_back();

    for (std::size_t i = 0; i < 6; i++)
        state[6 * index + i] = state[state.size() - 6 + i];
    state.resize(6 * m_bodies.size());

    validate();
    events.on_late_body_removal(std::move(index)); // It just made me do this...
    return true;
}

bool world2D::remove_body(const body2D &body)
{
    return remove_body(body.index);
}

bool world2D::remove_body(kit::uuid id)
{
    for (const body2D &body : m_bodies)
        if (body.id == id)
            return remove_body(body.index);
    return false;
}

bool world2D::remove_behaviour(std::size_t index)
{
    if (index >= m_behaviours.size())
    {
        KIT_WARN("Behaviour index exceeds array bounds. Aborting... - index: {0}, size: {1}", index,
                 m_behaviours.size())
        return false;
    }
    events.on_behaviour_removal(*m_behaviours[index]);
    m_behaviours.erase(m_behaviours.begin() + (long)index);
    return true;
}
bool world2D::remove_behaviour(const behaviour2D *bhv)
{
    for (auto it = m_behaviours.begin(); it != m_behaviours.end(); ++it)
        if (it->get() == bhv)
        {
            events.on_behaviour_removal(*bhv);
            m_behaviours.erase(it);
            return true;
        }
    return false;
}
bool world2D::remove_behaviour(const std::string &name)
{
    for (auto it = m_behaviours.begin(); it != m_behaviours.end(); ++it)
        if ((*it)->id == name)
        {
            events.on_behaviour_removal(**it);
            m_behaviours.erase(it);
            return true;
        }
    return false;
}

bool world2D::remove_spring(std::size_t index)
{
    if (index >= m_springs.size())
    {
        KIT_WARN("Spring index exceeds array bounds. Aborting... - index: {0}, size: {1}", index, m_springs.size())
        return false;
    }
    events.on_early_spring_removal(m_springs[index]);
    if (index != m_springs.size() - 1)
    {
        m_springs[index] = m_springs.back();
        m_springs[index].index = index;
    }
    m_springs.pop_back();
    events.on_late_spring_removal(std::move(index));
    return true;
}
bool world2D::remove_spring(const spring2D &sp)
{
    return remove_spring(sp.index);
}
bool world2D::remove_spring(kit::uuid id)
{
    for (const spring2D &sp : m_springs)
        if (sp.id == id)
            return remove_spring(sp.index);
    return false;
}

bool world2D::remove_constraint(std::size_t index)
{
    return m_constraint_manager.remove_constraint(index, events.on_constraint_removal);
}
bool world2D::remove_constraint(const constraint2D *ctr)
{
    return m_constraint_manager.remove_constraint(ctr, events.on_constraint_removal);
}
bool world2D::remove_constraint(kit::uuid id)
{
    return m_constraint_manager.remove_constraint(id, events.on_constraint_removal);
}

void world2D::clear_bodies()
{
    for (std::size_t i = m_bodies.size() - 1; i < m_bodies.size(); i--)
        remove_body(i);
}
void world2D::clear_behaviours()
{
    for (const auto &bhv : m_behaviours)
        events.on_behaviour_removal(*bhv);
    m_behaviours.clear();
}
void world2D::clear_springs()
{
    std::vector<std::size_t> indices;
    indices.reserve(m_springs.size());
    for (const spring2D &sp : m_springs)
    {
        events.on_early_spring_removal(sp);
        indices.push_back(sp.index);
    }
    m_springs.clear();
    for (std::size_t idx : indices)
        events.on_late_spring_removal(std::move(idx));
}
void world2D::clear_constraints()
{
    m_constraint_manager.clear_constraints(events.on_constraint_removal);
}
void world2D::clear()
{
    m_behaviours.clear();
    m_springs.clear();
    m_constraint_manager.clear_constraints(events.on_constraint_removal);
    clear_bodies();
}

float world2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const body2D &body : m_bodies)
        ke += body.kinetic_energy();
    return ke;
}
float world2D::potential_energy() const
{
    float pot = 0.f;
    for (const auto &bhv : m_behaviours)
        if (bhv->enabled)
            pot += bhv->potential_energy();
    for (const spring2D &sp : m_springs)
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
        vars.size() == 6 * m_bodies.size(),
        "State vector size must be exactly 6 times greater than the body array size - vars: {0}, body array: {1}",
        vars.size(), m_bodies.size())

    reset_bodies_simulation_forces();
    retrieve_data_from_state_variables(vars);
    apply_world_behaviours_and_springs();
    apply_added_forces();

    if (enable_collisions)
    {
        const auto &collisions = m_collision_detection->cached_collisions();
        m_collision_solver->solve(collisions);
    }

    m_constraint_manager.solve_constraints();
    return create_state_derivative();
}

template <typename T> static std::optional<std::size_t> index_from_id(const kit::uuid id, const std::vector<T> &vec)
{
    constexpr bool is_ctr = std::is_same_v<T, kit::scope<constraint2D>>;
    for (std::size_t i = 0; i < vec.size(); i++)
        if constexpr (!is_ctr)
        {
            if (vec[i].id == id)
                return i;
        }
        else
        {
            if (vec[i]->id == id)
                return i;
        }
    return {};
}

body2D::const_ptr world2D::body_from_id(const kit::uuid id) const
{
    const auto index = index_from_id(id, m_bodies);
    return index ? (*this)[index.value()] : nullptr;
}
body2D::ptr world2D::body_from_id(const kit::uuid id)
{
    const auto index = index_from_id(id, m_bodies);
    return index ? (*this)[index.value()] : nullptr;
}

spring2D::const_ptr world2D::spring_from_id(const kit::uuid id) const
{
    const auto index = index_from_id(id, m_springs);
    return index ? spring(index.value()) : nullptr;
}
spring2D::ptr world2D::spring_from_id(const kit::uuid id)
{
    const auto index = index_from_id(id, m_springs);
    return index ? spring(index.value()) : nullptr;
}

const constraint2D *world2D::constraint_from_id(const kit::uuid id) const
{
    const auto index = index_from_id(id, m_constraint_manager.constraints());
    return index ? m_constraint_manager.constraints()[index.value()].get() : nullptr;
}
constraint2D *world2D::constraint_from_id(const kit::uuid id)
{
    const auto index = index_from_id(id, m_constraint_manager.constraints());
    return index ? m_constraint_manager.constraints()[index.value()].get() : nullptr;
}

template <> behaviour2D *world2D::behaviour_from_name(const std::string &name) const
{
    for (const auto &bhv : m_behaviours)
        if (name == bhv->id)
            return bhv.get();
    return nullptr;
}

body2D::const_ptr world2D::operator[](const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_bodies.size())
    return {&m_bodies, index};
}
body2D::ptr world2D::operator[](const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_bodies.size())
    return {&m_bodies, index};
}

std::vector<body2D::const_ptr> world2D::operator[](const geo::aabb2D &aabb) const
{
    std::vector<body2D::const_ptr> in_area;
    in_area.reserve(m_bodies.size() / 2);

    for (const body2D &body : m_bodies)
        if (geo::intersect(body.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_bodies, body.index);
    return in_area;
}
std::vector<body2D::ptr> world2D::operator[](const geo::aabb2D &aabb)
{
    std::vector<body2D::ptr> in_area;
    in_area.reserve(m_bodies.size() / 2);
    for (const body2D &body : m_bodies)
        if (geo::intersect(body.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_bodies, body.index);
    return in_area;
}

std::vector<const constraint2D *> world2D::constraints_from_ids(const std::vector<kit::uuid> &ids) const
{
    KIT_ASSERT_ERROR(std::unordered_set<kit::uuid>(ids.begin(), ids.end()).size() == ids.size(),
                     "IDs list must not contain duplicates!")

    std::vector<const constraint2D *> constraints;
    if (ids.empty())
        return constraints;

    constraints.reserve(m_constraint_manager.constraints().size());
    for (const auto &ctr : m_constraint_manager.constraints())
    {
        bool found_match = true;
        for (kit::uuid id : ids)
            if (!ctr->contains(id))
            {
                found_match = false;
                break;
            }
        if (found_match)
            constraints.push_back(ctr.get());
    }
    return constraints;
}
std::vector<constraint2D *> world2D::constraints_from_ids(const std::vector<kit::uuid> &ids)
{
    KIT_ASSERT_ERROR(std::unordered_set<kit::uuid>(ids.begin(), ids.end()).size() == ids.size(),
                     "IDs list must not contain duplicates!")

    std::vector<constraint2D *> constraints;
    if (ids.empty())
        return constraints;

    constraints.reserve(m_constraint_manager.constraints().size());
    for (const auto &ctr : m_constraint_manager.constraints())
    {
        bool found_match = true;
        for (kit::uuid id : ids)
            if (!ctr->contains(id))
            {
                found_match = false;
                break;
            }
        if (found_match)
            constraints.push_back(ctr.get());
    }
    return constraints;
}

const std::vector<kit::scope<behaviour2D>> &world2D::behaviours() const
{
    return m_behaviours;
}
const std::vector<kit::scope<constraint2D>> &world2D::constraints() const
{
    return m_constraint_manager.constraints();
}

const std::vector<spring2D> &world2D::springs() const
{
    return m_springs;
}
spring2D::const_ptr world2D::spring(std::size_t index) const
{
    return {&m_springs, index};
}

std::vector<spring2D::const_ptr> world2D::springs_from_ids(const kit::uuid id1, const kit::uuid id2) const
{
    std::vector<spring2D::const_ptr> springs;
    springs.reserve(m_springs.size());

    for (const spring2D &sp : m_springs)
        if ((sp.joint.body1().id == id1 && sp.joint.body2().id == id2) ||
            (sp.joint.body1().id == id2 && sp.joint.body2().id == id1))
            springs.emplace_back(&m_springs, sp.index);
    return springs;
}

kit::vector_view<spring2D> world2D::springs()
{
    return m_springs;
}
spring2D::ptr world2D::spring(std::size_t index)
{
    return {&m_springs, index};
}

std::vector<spring2D::ptr> world2D::springs_from_ids(const kit::uuid id1, const kit::uuid id2)
{
    std::vector<spring2D::ptr> springs;
    springs.reserve(m_springs.size());

    for (const spring2D &sp : m_springs)
        if ((sp.joint.body1().id == id1 && sp.joint.body2().id == id2) ||
            (sp.joint.body1().id == id2 && sp.joint.body2().id == id1))
            springs.emplace_back(&m_springs, sp.index);
    return springs;
}

body2D::const_ptr world2D::operator[](const glm::vec2 &point) const
{
    const geo::aabb2D aabb = point;
    for (const body2D &body : m_bodies)
        if (geo::intersect(body.shape().bounding_box(), aabb))
            return {&m_bodies, body.index};
    return nullptr;
}
body2D::ptr world2D::operator[](const glm::vec2 &point)
{
    const geo::aabb2D aabb = point;
    for (const body2D &body : m_bodies)
        if (geo::intersect(body.shape().bounding_box(), aabb))
            return {&m_bodies, body.index};
    return nullptr;
}

const std::vector<body2D> &world2D::bodies() const
{
    return m_bodies;
}
kit::vector_view<body2D> world2D::bodies()
{
    return m_bodies;
}
std::size_t world2D::size() const
{
    return m_bodies.size();
}

float world2D::elapsed() const
{
    return m_elapsed;
}
#ifdef KIT_USE_YAML_CPP
YAML::Node world2D::serializer::encode(const world2D &world) const
{
    YAML::Node node;
    for (const ppx::body2D &body : world.bodies())
        node["Bodies"].push_back(body);

    for (const ppx::spring2D &sp : world.springs())
        node["Springs"].push_back(sp);
    for (const auto &ctr : world.constraints())
    {
        YAML::Node child;
        child[ctr->name] = *ctr;
        node["Constraints"].push_back(child);
    }

    for (const auto &bhv : world.behaviours())
        node["Behaviours"][bhv->id] = *bhv;

    node["Integrator"] = world.integrator;
    node["Elapsed"] = world.elapsed();
    return node;
}
bool world2D::serializer::decode(const YAML::Node &node, world2D &world) const
{
    if (!node.IsMap() || node.size() < 3)
        return false;

    world.clear_bodies();
    world.integrator = node["Integrator"].as<rk::integrator>();
    world.integrator.state.clear();

    if (node["Bodies"])
        for (const YAML::Node &n : node["Bodies"])
            world.add_body(n.as<ppx::body2D>());

    if (node["Springs"])
        for (const YAML::Node &n : node["Springs"])
        {
            spring2D sp;
            sp.m_world = &world;
            n.as<spring2D>(sp);
            world.add_spring(sp);
        }

    if (node["Constraints"])
        for (const YAML::Node &n : node["Constraints"])
            if (n["Distance"])
            {
                distance_joint2D dj;
                dj.m_world = &world;
                n["Distance"].as<distance_joint2D>(dj);
                world.add_constraint<distance_joint2D>(dj);
            }

    if (node["Behaviours"])
        for (auto it = node["Behaviours"].begin(); it != node["Behaviours"].end(); ++it)
        {
            const auto bhv = world.behaviour_from_name<ppx::behaviour2D>(it->first.as<std::string>().c_str());
            if (bhv && node["Behaviours"][bhv->id])
                node["Behaviours"][bhv->id].as<ppx::behaviour2D>(*bhv);
        }

    world.m_elapsed = node["Elapsed"].as<float>();
    return true;
}
#endif
} // namespace ppx