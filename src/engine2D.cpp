#include "ppx/internal/pch.hpp"
#include "ppx/engine2D.hpp"
#include "geo/intersection.hpp"
#include "ppx/behaviours/behaviour2D.hpp"
#include "ppx/joints/revolute_joint2D.hpp"
#include <cstring>

namespace ppx
{
engine2D::engine2D(const rk::butcher_tableau &table, const std::size_t allocations)
    : m_collider(*this, 2 * allocations), m_compeller(*this, allocations, &m_events), m_integ(table)
{
    m_entities.reserve(allocations);
    m_integ.state().reserve(6 * allocations);
}

void engine2D::retrieve(const std::vector<float> &vars_buffer)
{
    KIT_PERF_FUNCTION()
    for (std::size_t i = 0; i < m_entities.size(); i++)
        m_entities[i].retrieve(vars_buffer);
}

void engine2D::retrieve()
{
    retrieve(m_integ.state().vars());
}

bool engine2D::raw_forward(float &timestep)
{
    const bool valid = m_integ.raw_forward(m_elapsed, timestep, *this);
    reset_entities();
    retrieve();
    m_collider.flush_collisions();
    return valid;
}
bool engine2D::reiterative_forward(float &timestep, const std::uint8_t reiterations)
{
    const bool valid = m_integ.reiterative_forward(m_elapsed, timestep, *this, reiterations);
    reset_entities();
    retrieve();
    m_collider.flush_collisions();
    return valid;
}
bool engine2D::embedded_forward(float &timestep)
{
    const bool valid = m_integ.embedded_forward(m_elapsed, timestep, *this);
    reset_entities();
    retrieve();
    m_collider.flush_collisions();
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
    for (std::size_t i = 0; i < m_entities.size(); i++)
    {
        const std::size_t index = 6 * i;
        const glm::vec2 &vel = m_entities[i].vel();
        const float angvel = m_entities[i].angvel();
        stchanges[index] = vel.x;
        stchanges[index + 1] = vel.y;
        stchanges[index + 2] = angvel;
        if (m_entities[i].kinematic())
        {
            const glm::vec2 &force = m_entities[i].added_force();
            const float torque = m_entities[i].added_torque();
            load_force(stchanges, force, torque, index);
        }
    }
}

void engine2D::validate()
{
    m_collider.validate();
    m_compeller.validate();
    for (const auto &bhv : m_behaviours)
        bhv->validate();
    for (auto it = m_springs.begin(); it != m_springs.end();)
        if (!it->valid())
        {
            m_events.on_spring_removal(*it);
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
            for (const auto &e : bhv->entities())
            {
                if (!e->kinematic())
                    continue;
                const auto [force, torque] = bhv->force(*e);
                load_force(stchanges, force, torque, 6 * e->index());
            }
    for (const spring2D &s : m_springs)
    {
        const std::size_t index1 = 6 * s.e1()->index(), index2 = 6 * s.e2()->index();
        const auto [force, t1, t2] = s.force();
        if (s.e1()->kinematic())
            load_force(stchanges, force, t1, index1);
        if (s.e2()->kinematic())
            load_force(stchanges, -force, t2, index2);
    }
}

kit::stack_vector<float> engine2D::effective_inverse_masses() const
{
    KIT_PERF_FUNCTION()
    kit::stack_vector<float> inv_masses;
    inv_masses.reserve(3 * m_entities.size());
    for (std::size_t i = 0; i < m_entities.size(); i++)
    {
        const float inv_mass = m_entities[i].kinematic() ? m_entities[i].inverse_mass() : 0.f,
                    inv_inertia = m_entities[i].kinematic() ? m_entities[i].inverse_inertia() : 0.f;
        inv_masses.insert(inv_masses.end(), {inv_mass, inv_mass, inv_inertia});
    }
    return inv_masses;
}

void engine2D::reset_entities()
{
    for (entity2D &e : m_entities)
    {
        e.m_added_force = glm::vec2(0.f);
        e.m_added_torque = 0.f;
        e.events().reset();
    }
}

entity2D::ptr engine2D::process_entity_addition(entity2D &e)
{
    rk::state &state = m_integ.state();
    e.m_state = &state;

    const entity2D::ptr e_ptr = {&m_entities, m_entities.size() - 1};
    const glm::vec2 &pos = e.pos(), &vel = e.vel();
    state.append({pos.x, pos.y, e.angpos(), vel.x, vel.y, e.angvel()});
    e.retrieve();
    m_collider.add_entity_intervals(e_ptr);

    KIT_INFO("Added entity with index {0} and id {1}.", e.index(), (std::uint64_t)e.id())
#ifdef DEBUG
    for (std::size_t i = 0; i < m_entities.size() - 1; i++)
        KIT_ASSERT_CRITICAL(m_entities[i].id() != e.id(),
                            "Entity with index {0} has the same id as entity with index {1}", i, e.index())
#endif
    m_events.on_entity_addition(e_ptr);
    return e_ptr;
}

bool engine2D::remove_entity(std::size_t index)
{
    if (index >= m_entities.size())
    {
        KIT_WARN("Index exceeds entity array bounds. Aborting... - index: {0}, size: {1}", index, m_entities.size())
        return false;
    }
    KIT_INFO("Removing entity with index {0} and id {1}", index, m_entities[index].id())

    m_events.on_early_entity_removal(m_entities[index]);
    rk::state &state = m_integ.state();
    m_entities.erase(index);

    for (std::size_t i = 0; i < 6; i++)
        state[6 * index + i] = state[state.size() - 6 + i];
    state.resize(6 * m_entities.size());

    validate();
    m_events.on_late_entity_removal(std::move(index)); // It just made me do this...
    return true;
}

bool engine2D::remove_entity(const entity2D &e)
{
    return remove_entity(e.index());
}

bool engine2D::remove_behaviour(const behaviour2D *bhv)
{
    for (auto it = m_behaviours.begin(); it != m_behaviours.end(); ++it)
        if (it->get() == bhv)
        {
            m_events.on_behaviour_removal(*bhv);
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
            m_events.on_behaviour_removal(**it);
            m_behaviours.erase(it);
            return true;
        }
    return false;
}

bool engine2D::remove_spring(std::size_t index)
{
    if (index >= m_springs.size())
    {
        KIT_WARN("Index exceeds entity array bounds. Aborting... - index: {0}, size: {1}", index, m_springs.size())
        return false;
    }
    m_events.on_spring_removal(m_springs[index]);
    m_springs.erase(m_springs.begin() + (long)index);
    return true;
}
bool engine2D::remove_spring(const spring2D &sp)
{
    for (std::size_t i = 0; i < m_springs.size(); i++)
        if (m_springs[i] == sp)
            return remove_spring(i);
    return false;
}

void engine2D::clear_entities()
{
    for (std::size_t i = m_entities.size() - 1; i < m_entities.size(); i--)
        remove_entity(i);
}
void engine2D::clear_behaviours()
{
    for (const auto &bhv : m_behaviours)
        m_events.on_behaviour_removal(*bhv);
    m_behaviours.clear();
}
void engine2D::clear_springs()
{
    for (const spring2D &sp : m_springs)
        m_events.on_spring_removal(sp);
    m_springs.clear();
}
void engine2D::clear_constraints()
{
    m_compeller.clear_constraints();
}
void engine2D::clear()
{
    m_behaviours.clear();
    m_springs.clear();
    m_compeller.clear_constraints();
    clear_entities();
}

float engine2D::kinetic_energy() const
{
    float ke = 0.f;
    for (const entity2D &e : m_entities)
        ke += e.kinetic_energy();
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
        vars.size() == 6 * m_entities.size(),
        "State vector size must be exactly 6 times greater than the entity array size - vars: {0}, entity array: {1}",
        vars.size(), m_entities.size())
    std::vector<float> stchanges(vars.size(), 0.f);

    retrieve(vars);
    load_velocities_and_added_forces(stchanges);
    load_interactions_and_externals(stchanges);
    const kit::stack_vector<float> inv_masses = effective_inverse_masses();

    m_collider.solve_and_load_collisions(stchanges);
    m_compeller.solve_and_load_constraints(stchanges, inv_masses);
    for (std::size_t i = 0; i < m_entities.size(); i++)
    {
        stchanges[6 * i + 3] *= inv_masses[3 * i];
        stchanges[6 * i + 4] *= inv_masses[3 * i + 1];
        stchanges[6 * i + 5] *= inv_masses[3 * i + 2];
    }
    return stchanges;
}

std::optional<std::size_t> engine2D::index_from_id(const kit::uuid id) const
{
    for (std::size_t i = 0; i < m_entities.size(); i++)
        if (m_entities[i].id() == id)
            return i;
    return {};
}

entity2D::const_ptr engine2D::from_id(kit::uuid id) const
{
    const auto index = index_from_id(id);
    return index ? (*this)[index.value()] : nullptr;
}

entity2D::ptr engine2D::from_id(kit::uuid id)
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

entity2D::const_ptr engine2D::operator[](const std::size_t index) const
{
    KIT_ASSERT_ERROR(index < m_entities.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_entities.size())
    return {&m_entities, index};
}
entity2D::ptr engine2D::operator[](const std::size_t index)
{
    KIT_ASSERT_ERROR(index < m_entities.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                     m_entities.size())
    return {&m_entities, index};
}

std::vector<entity2D::const_ptr> engine2D::operator[](const geo::aabb2D &aabb) const
{
    std::vector<entity2D::const_ptr> in_area;
    in_area.reserve(m_entities.size() / 2);

    for (const entity2D &e : m_entities)
        if (geo::intersect(e.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_entities, e.index());
    return in_area;
}
std::vector<entity2D::ptr> engine2D::operator[](const geo::aabb2D &aabb)
{
    std::vector<entity2D::ptr> in_area;
    in_area.reserve(m_entities.size() / 2);
    for (const entity2D &e : m_entities)
        if (geo::intersect(e.shape().bounding_box(), aabb))
            in_area.emplace_back(&m_entities, e.index());
    return in_area;
}

const std::vector<kit::scope<behaviour2D>> &engine2D::behaviours() const
{
    return m_behaviours;
}
const kit::track_vector<spring2D> &engine2D::springs() const
{
    return m_springs;
}
spring2D::const_ptr engine2D::spring(std::size_t index) const
{
    return {&m_springs, index};
}

kit::vector_view<kit::scope<behaviour2D>> engine2D::behaviours()
{
    return m_behaviours;
}
kit::track_vector_view<spring2D> engine2D::springs()
{
    return m_springs;
}
spring2D::ptr engine2D::spring(std::size_t index)
{
    return {&m_springs, index};
}

entity2D::const_ptr engine2D::operator[](const glm::vec2 &point) const
{
    const geo::aabb2D aabb = point;
    for (const entity2D &e : m_entities)
        if (geo::intersect(e.shape().bounding_box(), aabb))
            return {&m_entities, e.index()};
    return nullptr;
}
entity2D::ptr engine2D::operator[](const glm::vec2 &point)
{
    const geo::aabb2D aabb = point;
    for (const entity2D &e : m_entities)
        if (geo::intersect(e.shape().bounding_box(), aabb))
            return {&m_entities, e.index()};
    return nullptr;
}

const kit::track_vector<entity2D> &engine2D::entities() const
{
    return m_entities;
}
kit::track_vector_view<entity2D> engine2D::entities()
{
    return m_entities;
}
std::size_t engine2D::size() const
{
    return m_entities.size();
}

const rk::integrator &engine2D::integrator() const
{
    return m_integ;
}
rk::integrator &engine2D::integrator()
{
    return m_integ;
}

const collider2D &engine2D::collider() const
{
    return m_collider;
}
collider2D &engine2D::collider()
{
    return m_collider;
}

const compeller2D &engine2D::compeller() const
{
    return m_compeller;
}
compeller2D &engine2D::compeller()
{
    return m_compeller;
}

engine_events &engine2D::events()
{
    return m_events;
}

float engine2D::elapsed() const
{
    return m_elapsed;
}
#ifdef KIT_USE_YAML_CPP
YAML::Node engine2D::serializer::encode(const engine2D &eng) const
{
    YAML::Node node;
    for (const ppx::entity2D &e : eng.entities())
        node["Entities"].push_back(e);
    node["Collider"] = eng.collider();
    for (const ppx::spring2D &sp : eng.springs())
        node["Springs"].push_back(sp);
    for (const auto &ctr : eng.compeller().constraints())
    {
        const auto revjoint = dynamic_cast<const ppx::revolute_joint2D *>(ctr.get());
        if (revjoint)
            node["Rigid bars"].push_back(*revjoint);
    }
    for (const auto &bhv : eng.behaviours())
        node["Behaviours"][bhv->id()] = *bhv;

    // Save checkpoint?
    node["Integrator"] = eng.integrator();
    node["Elapsed"] = eng.elapsed();
    return node;
}
bool engine2D::serializer::decode(const YAML::Node &node, engine2D &eng) const
{
    if (!node.IsMap() || node.size() != 7)
        return false;

    eng.clear_entities();
    eng.m_integ = node["Integrator"].as<rk::integrator>();
    eng.m_integ.state().clear();

    for (const YAML::Node &n : node["Entities"])
        eng.add_entity(n.as<ppx::entity2D>());

    node["Collider"].as<ppx::collider2D>(eng.collider());
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

    for (const YAML::Node &n : node["Rigid bars"])
    {
        const std::size_t idx1 = n["Index1"].as<std::size_t>(), idx2 = n["Index2"].as<std::size_t>();
        if (n["Anchor1"])
        {
            const auto revjoint = eng.compeller().add_constraint<ppx::revolute_joint2D>(
                eng[idx1], eng[idx2], n["Anchor1"].as<glm::vec2>(), n["Anchor2"].as<glm::vec2>());
            n.as<ppx::revolute_joint2D>(*revjoint);
            continue;
        }
        const auto revjoint = eng.compeller().add_constraint<ppx::revolute_joint2D>(eng[idx1], eng[idx2]);
        n.as<ppx::revolute_joint2D>(*revjoint);
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