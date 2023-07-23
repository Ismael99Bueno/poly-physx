#include "ppx/internal/pch.hpp"
#include "ppx/engine2D.hpp"
#include "geo/intersection.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include <cstring>

namespace ppx
{
engine2D::engine2D(const rk::butcher_tableau &table, const std::size_t allocations)
    : collisions(*this, 2 * allocations), integrator(table), m_compeller(*this, allocations)
{
    m_bodies.reserve(allocations);
    integrator.state().reserve(6 * allocations);
}

void engine2D::retrieve(const std::vector<float> &vars_buffer)
{
    KIT_PERF_FUNCTION()
    for (std::size_t i = 0; i < m_bodies.size(); i++)
        m_bodies[i].retrieve(vars_buffer);
}

void engine2D::retrieve()
{
    retrieve(integrator.state().vars());
}

bool engine2D::raw_forward(const float timestep)
{
    const bool valid = integrator.raw_forward(m_elapsed, timestep, *this);
    reset_bodies();
    retrieve();
    collisions.flush_collisions();
    return valid;
}
bool engine2D::reiterative_forward(float &timestep, const std::uint8_t reiterations)
{
    const bool valid = integrator.reiterative_forward(m_elapsed, timestep, *this, reiterations);
    reset_bodies();
    retrieve();
    collisions.flush_collisions();
    return valid;
}
bool engine2D::embedded_forward(float &timestep)
{
    const bool valid = integrator.embedded_forward(m_elapsed, timestep, *this);
    reset_bodies();
    retrieve();
    collisions.flush_collisions();
    return valid;
}

static void load_force(std::vector<float> &stchanges, const glm::vec2 &force, float torque, std::size_t index)
{
    stchanges[index + 3] += force.x;
    stchanges[index + 4] += force.y;
    stchanges[index + 5] += torque;
}

void engine2D::load_velocities_and_added_forces(std::vector<float> &stchanges) const
{
    KIT_PERF_FUNCTION()
    for (std::size_t i = 0; i < m_bodies.size(); i++)
    {
        const std::size_t index = 6 * i;
        const glm::vec2 &vel = m_bodies[i].vel();
        const float angvel = m_bodies[i].angvel();
        stchanges[index] = vel.x;
        stchanges[index + 1] = vel.y;
        stchanges[index + 2] = angvel;
        if (m_bodies[i].kinematic())
        {
            const glm::vec2 &force = m_bodies[i].added_force();
            const float torque = m_bodies[i].added_torque();
            load_force(stchanges, force, torque, index);
        }
    }
}

void engine2D::validate()
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

void engine2D::load_interactions_and_externals(std::vector<float> &stchanges) const
{
    KIT_PERF_FUNCTION()
    for (const auto &bhv : m_behaviours)
        if (bhv->enabled())
            for (const auto &bd : bhv->bodies())
            {
                if (!bd->kinematic())
                    continue;
                const auto [force, torque] = bhv->force(*bd);
                load_force(stchanges, force, torque, 6 * bd->index());
            }
    for (const spring2D &s : m_springs)
    {
        const std::size_t index1 = 6 * s.bd1()->index(), index2 = 6 * s.bd2()->index();
        const auto [force, t1, t2] = s.force();
        if (s.bd1()->kinematic())
            load_force(stchanges, force, t1, index1);
        if (s.bd2()->kinematic())
            load_force(stchanges, -force, t2, index2);
    }
}

kit::stack_vector<float> engine2D::effective_inverse_masses() const
{
    KIT_PERF_FUNCTION()
    kit::stack_vector<float> inv_masses;
    inv_masses.reserve(3 * m_bodies.size());
    for (std::size_t i = 0; i < m_bodies.size(); i++)
    {
        const float inv_mass = m_bodies[i].kinematic() ? m_bodies[i].inverse_mass() : 0.f,
                    inv_inertia = m_bodies[i].kinematic() ? m_bodies[i].inverse_inertia() : 0.f;
        inv_masses.insert(inv_masses.end(), {inv_mass, inv_mass, inv_inertia});
    }
    return inv_masses;
}

void engine2D::reset_bodies()
{
    for (body2D &bd : m_bodies)
    {
        bd.m_added_force = glm::vec2(0.f);
        bd.m_added_torque = 0.f;
    }
}

body2D::ptr engine2D::process_body_addition(body2D &bd)
{
    rk::state &state = integrator.state();
    bd.m_state = &state;

    const body2D::ptr e_ptr = {&m_bodies, m_bodies.size() - 1};
    const glm::vec2 &pos = bd.pos(), &vel = bd.vel();
    state.append({pos.x, pos.y, bd.angpos(), vel.x, vel.y, bd.angvel()});
    bd.retrieve();
    collisions.add_body_intervals(e_ptr);

    KIT_INFO("Added body with index {0} and id {1}.", bd.index(), (std::uint64_t)bd.id())
#ifdef DEBUG
    for (std::size_t i = 0; i < m_bodies.size() - 1; i++)
        KIT_ASSERT_CRITICAL(m_bodies[i].id() != bd.id(), "Body with index {0} has the same id as body with index {1}",
                            i, bd.index())
#endif
    events.on_body_addition(e_ptr);
    return e_ptr;
}

bool engine2D::remove_body(std::size_t index)
{
    if (index >= m_bodies.size())
    {
        KIT_WARN("Body index exceeds array bounds. Aborting... - index: {0}, size: {1}", index, m_bodies.size())
        return false;
    }
    KIT_INFO("Removing body with index {0} and id {1}", index, m_bodies[index].id())

    events.on_early_body_removal(m_bodies[index]);
    rk::state &state = integrator.state();
    m_bodies.erase(index);

    for (std::size_t i = 0; i < 6; i++)
        state[6 * index + i] = state[state.size() - 6 + i];
    state.resize(6 * m_bodies.size());

    validate();
    events.on_late_body_removal(std::move(index)); // It just made me do this...
    return true;
}

bool engine2D::remove_body(const body2D &bd)
{
    return remove_body(bd.index());
}

bool engine2D::remove_body(kit::uuid id)
{
    for (const body2D &bd : m_bodies)
        if (bd.id() == id)
            return remove_body(bd.index());
    return false;
}

bool engine2D::remove_behaviour(std::size_t index)
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
bool engine2D::remove_behaviour(const behaviour2D *bhv)
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
bool engine2D::remove_behaviour(const std::string &name)
{
    for (auto it = m_behaviours.begin(); it != m_behaviours.end(); ++it)
        if ((*it)->id() == name)
        {
            events.on_behaviour_removal(**it);
            m_behaviours.erase(it);
            return true;
        }
    return false;
}

bool engine2D::remove_spring(std::size_t index)
{
    if (index >= m_springs.size())
    {
        KIT_WARN("Spring index exceeds array bounds. Aborting... - index: {0}, size: {1}", index, m_springs.size())
        return false;
    }
    events.on_spring_removal(m_springs[index]);
    m_springs.erase(m_springs.begin() + (long)index);
    return true;
}
bool engine2D::remove_spring(const spring2D &sp)
{
    return remove_spring(sp.index());
}
bool engine2D::remove_spring(kit::uuid id)
{
    for (const spring2D &sp : m_springs)
        if (sp.id() == id)
            return remove_spring(sp.index());
    return false;
}

bool engine2D::remove_constraint(std::size_t index)
{
    return m_compeller.remove_constraint(index, events.on_constraint_removal);
}
bool engine2D::remove_constraint(const constraint2D *ctr)
{
    return m_compeller.remove_constraint(ctr, events.on_constraint_removal);
}
bool engine2D::remove_constraint(kit::uuid id)
{
    return m_compeller.remove_constraint(id, events.on_constraint_removal);
}

void engine2D::clear_bodies()
{
    for (std::size_t i = m_bodies.size() - 1; i < m_bodies.size(); i--)
        remove_body(i);
}
void engine2D::clear_behaviours()
{
    for (const auto &bhv : m_behaviours)
        events.on_behaviour_removal(*bhv);
    m_behaviours.clear();
}
void engine2D::clear_springs()
{
    for (const spring2D &sp : m_springs)
        events.on_spring_removal(sp);
    m_springs.clear();
}
void engine2D::clear_constraints()
{
    m_compeller.clear_constraints(events.on_constraint_removal);
}
void engine2D::clear()
{
    m_behaviours.clear();
    m_springs.clear();
    m_compeller.clear_constraints(events.on_constraint_removal);
    clear_bodies();
}

float engine2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const body2D &bd : m_bodies)
        ke += bd.kinetic_energy();
    return ke;
}
float engine2D::potential_energy() const
{
    float pot = 0.f;
    for (const auto &bhv : m_behaviours)
        pot += bhv->potential_energy();
    for (const spring2D &sp : m_springs)
        pot += sp.potential_energy();
    return pot;
}
float engine2D::energy() const
{
    return kinetic_energy() + potential_energy();
}

std::vector<float> engine2D::operator()(const float t, const float dt, const std::vector<float> &vars)
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

std::optional<std::size_t> engine2D::index_from_id(const kit::uuid id) const
{
    for (std::size_t i = 0; i < m_bodies.size(); i++)
        if (m_bodies[i].id() == id)
            return i;
    return {};
}

body2D::const_ptr engine2D::from_id(kit::uuid id) const
{
    const auto index = index_from_id(id);
    return index ? (*this)[index.value()] : nullptr;
}

body2D::ptr engine2D::from_id(kit::uuid id)
{
    const auto index = index_from_id(id);
    return index ? (*this)[index.value()] : nullptr;
}

template <> behaviour2D *engine2D::behaviour_from_name(const std::string &name) const
{
    for (const auto &bhv : m_behaviours)
        if (name == bhv->id())
            return bhv.get();
    return nullptr;
}

body2D::const_ptr engine2D::operator[](const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_bodies.size())
    return {&m_bodies, index};
}
body2D::ptr engine2D::operator[](const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_bodies.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_bodies.size())
    return {&m_bodies, index};
}

std::vector<body2D::const_ptr> engine2D::operator[](const geo::aabb2D &aabb) const
{
    std::vector<body2D::const_ptr> in_area;
    in_area.reserve(m_bodies.size() / 2);

    for (const body2D &bd : m_bodies)
        if (geo::intersect(bd.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_bodies, bd.index());
    return in_area;
}
std::vector<body2D::ptr> engine2D::operator[](const geo::aabb2D &aabb)
{
    std::vector<body2D::ptr> in_area;
    in_area.reserve(m_bodies.size() / 2);
    for (const body2D &bd : m_bodies)
        if (geo::intersect(bd.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_bodies, bd.index());
    return in_area;
}

const std::vector<kit::scope<behaviour2D>> &engine2D::behaviours() const
{
    return m_behaviours;
}
const std::vector<kit::scope<constraint2D>> &engine2D::constraints() const
{
    return m_compeller.constraints();
}

const kit::track_vector<spring2D> &engine2D::springs() const
{
    return m_springs;
}
spring2D::const_ptr engine2D::spring(std::size_t index) const
{
    return {&m_springs, index};
}

kit::track_vector_view<spring2D> engine2D::springs()
{
    return m_springs;
}
spring2D::ptr engine2D::spring(std::size_t index)
{
    return {&m_springs, index};
}

body2D::const_ptr engine2D::operator[](const glm::vec2 &point) const
{
    const geo::aabb2D aabb = point;
    for (const body2D &bd : m_bodies)
        if (geo::intersect(bd.shape().bounding_box(), aabb))
            return {&m_bodies, bd.index()};
    return nullptr;
}
body2D::ptr engine2D::operator[](const glm::vec2 &point)
{
    const geo::aabb2D aabb = point;
    for (const body2D &bd : m_bodies)
        if (geo::intersect(bd.shape().bounding_box(), aabb))
            return {&m_bodies, bd.index()};
    return nullptr;
}

const kit::track_vector<body2D> &engine2D::bodies() const
{
    return m_bodies;
}
kit::track_vector_view<body2D> engine2D::bodies()
{
    return m_bodies;
}
std::size_t engine2D::size() const
{
    return m_bodies.size();
}

float engine2D::elapsed() const
{
    return m_elapsed;
}
#ifdef KIT_USE_YAML_CPP
YAML::Node engine2D::serializer::encode(const engine2D &eng) const
{
    YAML::Node node;
    for (const ppx::body2D &bd : eng.bodies())
        node["Entities"].push_back(bd);
    node["Collider"] = eng.collisions;

    for (const ppx::spring2D &sp : eng.springs())
        node["Springs"].push_back(sp);
    for (const auto &ctr : eng.constraints())
    {
        YAML::Node child;
        child[ctr->name()] = *ctr;
        node["Constraints"].push_back(child);
    }

    for (const auto &bhv : eng.behaviours())
        node["Behaviours"][bhv->id()] = *bhv;

    node["Integrator"] = eng.integrator;
    node["Elapsed"] = eng.elapsed();
    return node;
}
bool engine2D::serializer::decode(const YAML::Node &node, engine2D &eng) const
{
    if (!node.IsMap() || node.size() != 7)
        return false;

    eng.clear_bodies();
    eng.integrator = node["Integrator"].as<rk::integrator>();
    eng.integrator.state().clear();

    for (const YAML::Node &n : node["Entities"])
        eng.add_body(n.as<ppx::body2D>());

    node["Collider"].as<ppx::collider2D>(eng.collisions);
    for (const YAML::Node &n : node["Springs"])
    {
        const std::size_t idx1 = n["Index1"].as<std::size_t>(), idx2 = n["Index2"].as<std::size_t>();
        if (n["Anchor1"])
        {
            const ppx::spring2D::ptr sp =
                eng.add_spring(eng[idx1], eng[idx2], n["Anchor1"].as<glm::vec2>(), n["Anchor2"].as<glm::vec2>());
            n.as<ppx::spring2D>(*sp);
            continue;
        }
        const ppx::spring2D::ptr sp = eng.add_spring(eng[idx1], eng[idx2]);
        n.as<ppx::spring2D>(*sp);
    }

    for (const YAML::Node &n : node["Constraints"])
        if (n["Revolute"])
        {
            const YAML::Node revnode = n["Revolute"];
            const std::size_t idx1 = revnode["Index1"].as<std::size_t>(), idx2 = revnode["Index2"].as<std::size_t>();
            if (revnode["Anchor1"])
            {
                const auto revjoint = eng.add_constraint<ppx::revolute_joint2D>(
                    eng[idx1], eng[idx2], revnode["Anchor1"].as<glm::vec2>(), revnode["Anchor2"].as<glm::vec2>());

                revnode.as<ppx::revolute_joint2D>(*revjoint);
                continue;
            }
            const auto revjoint = eng.add_constraint<ppx::revolute_joint2D>(eng[idx1], eng[idx2]);
            revnode.as<ppx::revolute_joint2D>(*revjoint);
        }

    for (auto it = node["Behaviours"].begin(); it != node["Behaviours"].end(); ++it)
    {
        const auto bhv = eng.behaviour_from_name<ppx::behaviour2D>(it->first.as<std::string>().c_str());
        node["Behaviours"][bhv->id()].as<ppx::behaviour2D>(*bhv);
    }

    eng.m_elapsed = node["Elapsed"].as<float>();
    return true;
}
#endif
} // namespace ppx