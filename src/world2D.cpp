#include "ppx/internal/pch.hpp"
#include "ppx/world2D.hpp"
#include "geo/intersection.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include <cstring>

namespace ppx
{
world2D::world2D(const rk::butcher_tableau &table, const std::size_t allocations)
    : collisions(*this, 2 * allocations), integrator(table), m_compeller(*this, allocations)
{
    m_bodies.reserve(allocations);
    integrator.state.reserve(6 * allocations);
}

void world2D::retrieve(const std::vector<float> &vars_buffer)
{
    KIT_PERF_FUNCTION()
    for (std::size_t i = 0; i < m_bodies.size(); i++)
        m_bodies[i].retrieve(vars_buffer);
}

void world2D::retrieve()
{
    retrieve(integrator.state.vars());
}

bool world2D::raw_forward(const float timestep)
{
    const bool valid = integrator.raw_forward(m_elapsed, timestep, *this);
    reset_bodies();
    retrieve();
    collisions.flush_collisions();
    return valid;
}
bool world2D::reiterative_forward(float &timestep, const std::uint8_t reiterations)
{
    const bool valid = integrator.reiterative_forward(m_elapsed, timestep, *this, reiterations);
    reset_bodies();
    retrieve();
    collisions.flush_collisions();
    return valid;
}
bool world2D::embedded_forward(float &timestep)
{
    const bool valid = integrator.embedded_forward(m_elapsed, timestep, *this);
    reset_bodies();
    retrieve();
    collisions.flush_collisions();
    return valid;
}

static void load_force(std::vector<float> &stchanges, const glm::vec3 &force, std::size_t index)
{
    stchanges[index + 3] += force.x;
    stchanges[index + 4] += force.y;
    stchanges[index + 5] += force.z;
}

void world2D::load_velocities_and_added_forces(std::vector<float> &stchanges) const
{
    KIT_PERF_FUNCTION()
    for (std::size_t i = 0; i < m_bodies.size(); i++)
    {
        const std::size_t index = 6 * i;
        const glm::vec2 &velocity = m_bodies[i].velocity();
        const float angular_velocity = m_bodies[i].angular_velocity();
        stchanges[index] = velocity.x;
        stchanges[index + 1] = velocity.y;
        stchanges[index + 2] = angular_velocity;
        if (m_bodies[i].kinematic)
        {
            const glm::vec2 &force = m_bodies[i].added_force();
            const float torque = m_bodies[i].added_torque();
            load_force(stchanges, glm::vec3(force, torque), index);
        }
    }
}

void world2D::validate()
{
    collisions.validate();
    m_compeller.validate(events.on_constraint_removal);
    for (const auto &bhv : m_behaviours)
        bhv->validate();
    for (auto it = m_springs.begin(); it != m_springs.end();)
        if (!it->valid())
        {
            events.on_spring_removal(*it);
            it = m_springs.erase(it);
        }
        else
            ++it;
}

void world2D::load_interactions_and_externals(std::vector<float> &stchanges) const
{
    KIT_PERF_FUNCTION()
    for (const auto &bhv : m_behaviours)
        if (bhv->enabled)
            for (const auto &body : bhv->bodies())
            {
                if (!body->kinematic)
                    continue;
                const glm::vec3 force = bhv->force(*body);
                load_force(stchanges, force, 6 * body->index);
            }
    for (const spring2D &s : m_springs)
    {
        const std::size_t index1 = 6 * s.body1()->index, index2 = 6 * s.body2()->index;
        const glm::vec4 force = s.force();
        if (s.body1()->kinematic)
            load_force(stchanges, glm::vec3(force), index1);
        if (s.body2()->kinematic)
            load_force(stchanges, glm::vec3(-glm::vec2(force), force.w), index2);
    }
}

kit::stack_vector<float> world2D::effective_inverse_masses() const
{
    KIT_PERF_FUNCTION()
    kit::stack_vector<float> inv_masses;
    inv_masses.reserve(3 * m_bodies.size());
    for (std::size_t i = 0; i < m_bodies.size(); i++)
    {
        const float inv_mass = m_bodies[i].kinematic ? m_bodies[i].inverse_mass() : 0.f,
                    inv_inertia = m_bodies[i].kinematic ? m_bodies[i].inverse_inertia() : 0.f;
        inv_masses.insert(inv_masses.end(), {inv_mass, inv_mass, inv_inertia});
    }
    return inv_masses;
}

void world2D::reset_bodies()
{
    for (body2D &body : m_bodies)
    {
        body.m_added_force = glm::vec2(0.f);
        body.m_added_torque = 0.f;
    }
}

body2D::ptr world2D::process_body_addition(body2D &body)
{
    rk::state &state = integrator.state;
    body.m_state = &state;

    body.index = m_bodies.size() - 1;
    const body2D::ptr e_ptr = {&m_bodies, m_bodies.size() - 1};

    const kit::transform2D &transform = body.transform();
    const glm::vec2 &velocity = body.velocity();

    state.append({transform.position.x, transform.position.y, transform.rotation, velocity.x, velocity.y,
                  body.angular_velocity()});
    body.retrieve();
    collisions.add_body_intervals(e_ptr);

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
    events.on_spring_removal(m_springs[index]);
    if (index != m_springs.size() - 1)
    {
        m_springs[index] = m_springs.back();
        m_springs[index].index = index;
    }
    m_springs.pop_back();
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
    return m_compeller.remove_constraint(index, events.on_constraint_removal);
}
bool world2D::remove_constraint(const constraint2D *ctr)
{
    return m_compeller.remove_constraint(ctr, events.on_constraint_removal);
}
bool world2D::remove_constraint(kit::uuid id)
{
    return m_compeller.remove_constraint(id, events.on_constraint_removal);
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
    for (const spring2D &sp : m_springs)
        events.on_spring_removal(sp);
    m_springs.clear();
}
void world2D::clear_constraints()
{
    m_compeller.clear_constraints(events.on_constraint_removal);
}
void world2D::clear()
{
    m_behaviours.clear();
    m_springs.clear();
    m_compeller.clear_constraints(events.on_constraint_removal);
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
    std::vector<float> stchanges(vars.size(), 0.f);

    retrieve(vars);
    load_velocities_and_added_forces(stchanges);
    load_interactions_and_externals(stchanges);
    const kit::stack_vector<float> inv_masses = effective_inverse_masses();

    collisions.solve_and_load_collisions(stchanges);
    m_compeller.solve_and_load_constraints(stchanges, inv_masses);
    for (std::size_t i = 0; i < m_bodies.size(); i++)
    {
        stchanges[6 * i + 3] *= inv_masses[3 * i];
        stchanges[6 * i + 4] *= inv_masses[3 * i + 1];
        stchanges[6 * i + 5] *= inv_masses[3 * i + 2];
    }
    return stchanges;
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
    const auto index = index_from_id(id, m_compeller.constraints());
    return index ? m_compeller.constraints()[index.value()].get() : nullptr;
}
constraint2D *world2D::constraint_from_id(const kit::uuid id)
{
    const auto index = index_from_id(id, m_compeller.constraints());
    return index ? m_compeller.constraints()[index.value()].get() : nullptr;
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
#ifdef DEBUG
    KIT_ASSERT_ERROR(std::unordered_set<kit::uuid>(ids.begin(), ids.end()).size() == ids.size(),
                     "IDs list must not contain duplicates!")
#endif

    std::vector<const constraint2D *> constraints;
    if (ids.empty())
        return constraints;

    constraints.reserve(m_compeller.constraints().size());
    for (const auto &ctr : m_compeller.constraints())
    {
        if (ctr->size() != ids.size())
            continue;

        bool found_match = true;
        for (std::size_t i = 0; i < ctr->size(); i++)
            if (std::find(ids.begin(), ids.end(), ctr->body(i)->id) == ids.end())
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
#ifdef DEBUG
    KIT_ASSERT_ERROR(std::unordered_set<kit::uuid>(ids.begin(), ids.end()).size() == ids.size(),
                     "IDs list must not contain duplicates!")
#endif

    std::vector<constraint2D *> constraints;
    if (ids.empty())
        return constraints;

    constraints.reserve(m_compeller.constraints().size());
    for (const auto &ctr : m_compeller.constraints())
    {
        if (ctr->size() != ids.size())
            continue;

        bool found_match = true;
        for (std::size_t i = 0; i < ctr->size(); i++)
            if (std::find(ids.begin(), ids.end(), ctr->body(i)->id) == ids.end())
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
    return m_compeller.constraints();
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
        if ((sp.body1().id == id1 && sp.body2().id == id2) || (sp.body1().id == id2 && sp.body2().id == id1))
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
        if ((sp.body1().id == id1 && sp.body2().id == id2) || (sp.body1().id == id2 && sp.body2().id == id1))
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
    node["Collider"] = world.collisions;

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

    node["Collider"].as<ppx::collider2D>(world.collisions);

    if (node["Springs"])
        for (const YAML::Node &n : node["Springs"])
        {
            const std::size_t idx1 = n["Index1"].as<std::size_t>(), idx2 = n["Index2"].as<std::size_t>();
            if (n["Anchor1"])
            {
                const ppx::spring2D::ptr sp = world.add_spring(world[idx1], world[idx2], n["Anchor1"].as<glm::vec2>(),
                                                               n["Anchor2"].as<glm::vec2>());
                n.as<ppx::spring2D>(*sp);
                continue;
            }
            const ppx::spring2D::ptr sp = world.add_spring(world[idx1], world[idx2]);
            n.as<ppx::spring2D>(*sp);
        }

    if (node["Constraints"])
        for (const YAML::Node &n : node["Constraints"])
            if (n["Revolute"])
            {
                const YAML::Node jointnode = n["Revolute"]["Joint2D"];
                const std::size_t idx1 = jointnode["Index1"].as<std::size_t>(),
                                  idx2 = jointnode["Index2"].as<std::size_t>();
                if (jointnode["Anchor1"])
                {
                    const auto revjoint = world.add_constraint<ppx::revolute_joint2D>(
                        world[idx1], world[idx2], jointnode["Anchor1"].as<glm::vec2>(),
                        jointnode["Anchor2"].as<glm::vec2>());

                    n["Revolute"].as<ppx::revolute_joint2D>(*revjoint);
                    continue;
                }
                const auto revjoint = world.add_constraint<ppx::revolute_joint2D>(world[idx1], world[idx2]);
                n["Revolute"].as<ppx::revolute_joint2D>(*revjoint);
            }

    if (node["Behaviours"])
        for (auto it = node["Behaviours"].begin(); it != node["Behaviours"].end(); ++it)
        {
            const auto bhv = world.behaviour_from_name<ppx::behaviour2D>(it->first.as<std::string>().c_str());
            node["Behaviours"][bhv->id].as<ppx::behaviour2D>(*bhv);
        }

    world.m_elapsed = node["Elapsed"].as<float>();
    return true;
}
#endif
} // namespace ppx