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
    if (!m_asleep)
        return;
    m_asleep = false;
    m_time_still = 0.f;
}
bool island2D::asleep() const
{
    return m_asleep && world.islands.params.enable_sleep;
}
bool island2D::about_to_sleep() const
{
    const float percent = 0.35f;
    return m_time_still > percent * world.islands.params.sleep_time_threshold;
}
bool island2D::evaluate_split_candidate()
{
    if (m_may_split)
        return true;
    if (is_void() || m_merged || !m_lost_contact || (!m_asleep && about_to_sleep()))
    {
        m_split_points = 0;
        return false;
    }
    return m_split_points++ > world.islands.params.steps_to_split;
}

void island2D::remove_body(body2D *body)
{
    for (std::size_t i = 0; i < m_bodies.size(); i++)
        if (m_bodies[i] == body)
        {
            m_bodies.erase(m_bodies.begin() + i);
            body->meta.island = nullptr;
            awake();
            m_may_split = true;
            if (no_bodies())
                world.islands.remove(this);
            return;
        }
    KIT_WARN("Body not found in island");
}

void island2D::merge(island2D &island)
{
    KIT_PERF_SCOPE("ppx::island2D::merge")
    for (body2D *body : island.m_bodies)
    {
        KIT_ASSERT_ERROR(body->is_dynamic(), "Body must be dynamic")
        KIT_ASSERT_ERROR(std::find(m_bodies.begin(), m_bodies.end(), body) == m_bodies.end(), "Body already in island")
        m_bodies.push_back(body);
        body->meta.island = this;
    }
    m_actuators.insert(m_actuators.end(), island.m_actuators.begin(), island.m_actuators.end());
    m_constraints.insert(m_constraints.end(), island.m_constraints.begin(), island.m_constraints.end());
    m_contacts.insert(m_contacts.end(), island.m_contacts.begin(), island.m_contacts.end());
    island.m_merged = true;
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

bool island2D::checksum() const
{
    const std::unordered_set<const body2D *> bodies(m_bodies.begin(), m_bodies.end());
    const std::unordered_set<const actuator2D *> actuators(m_actuators.begin(), m_actuators.end());
    const std::unordered_set<const constraint2D *> constraints(m_constraints.begin(), m_constraints.end());
    const std::unordered_set<const contact2D *> contacts(m_contacts.begin(), m_contacts.end());

    std::unordered_set<const actuator2D *> body_actuators;
    std::unordered_set<const constraint2D *> body_constraints;
    std::unordered_set<const contact2D *> body_contacts;
    for (const body2D *body : m_bodies)
    {
        if (!body->is_dynamic())
        {
            KIT_ERROR("Island checksum failed: Body is not dynamic")
            return false;
        }
        if (body->meta.island != this)
        {
            KIT_ERROR("Island checkusm failed: Body island mismatch")
            return false;
        }

        for (const joint2D *joint : body->meta.joints)
        {
            if (auto act = dynamic_cast<const actuator2D *>(joint))
            {
                if (!actuators.contains(act))
                {
                    KIT_ERROR("Island checksum failed: Actuator not found in actuator list")
                    return false;
                }
                body_actuators.insert(act);
            }
            else if (auto con = dynamic_cast<const constraint2D *>(joint))
            {
                if (!constraints.contains(con))
                {
                    KIT_ERROR("Island checksum failed: Constraint not found in constraint list")
                    return false;
                }
                body_constraints.insert(con);
            }
            if ((joint->body1()->is_dynamic() && !bodies.contains(joint->body1())) ||
                (joint->body2()->is_dynamic() && !bodies.contains(joint->body2())))
            {
                KIT_ERROR("Island checksum failed: A body's joint is not contained in the island's body list")
                return false;
            }
        }
        for (const contact2D *contact : body->meta.contacts)
        {
            if (!contacts.contains(contact))
            {
                KIT_ERROR("Island checksum failed: Contact not found in contact list")
                return false;
            }
            body_contacts.insert(contact);
            if (auto act = dynamic_cast<const actuator2D *>(contact))
            {
                if (!actuators.contains(act))
                {
                    KIT_ERROR("Island checksum failed: Actuator not found in actuator list")
                    return false;
                }
                body_actuators.insert(act);
            }
            else if (auto con = dynamic_cast<const constraint2D *>(contact))
            {
                if (!constraints.contains(con))
                {
                    KIT_ERROR("Island checksum failed: Constraint not found in constraint list")
                    return false;
                }
                body_constraints.insert(con);
            }
            if ((contact->body1()->is_dynamic() && !bodies.contains(contact->body1())) ||
                (contact->body2()->is_dynamic() && !bodies.contains(contact->body2())))
            {
                KIT_ERROR("Island checksum failed: A body's contact is not contained in the island's body list")
                return false;
            }
        }
    }

    KIT_ASSERT_ERROR(body_actuators.size() == m_actuators.size(), "Island checksum failed: Actuator count mismatch")
    KIT_ASSERT_ERROR(body_constraints.size() == m_constraints.size(),
                     "Island checksum failed: Constraint count mismatch")
    KIT_ASSERT_ERROR(body_contacts.size() == m_contacts.size(), "Island checksum failed: Contact count mismatch")

    KIT_ASSERT_ERROR(bodies.size() == m_bodies.size(), "Island checksum failed: Duplicate bodies found")
    KIT_ASSERT_ERROR(actuators.size() == m_actuators.size(), "Island checksum failed: Duplicate actuators found")
    KIT_ASSERT_ERROR(constraints.size() == m_constraints.size(), "Island checksum failed: Duplicate constraints found")
    KIT_ASSERT_ERROR(contacts.size() == m_contacts.size(), "Island checksum failed: Duplicate contacts found")

    return bodies.size() == m_bodies.size() && actuators.size() == m_actuators.size() &&
           constraints.size() == m_constraints.size() && contacts.size() == m_contacts.size() &&
           body_actuators.size() == m_actuators.size() && body_constraints.size() == m_constraints.size() &&
           body_contacts.size() == m_contacts.size();
}

void island2D::solve()
{
    collect_active_elements_and_call_pre_solve();
    for (actuator2D *actuator : m_active_actuators)
        actuator->solve();

    for (body2D *body : m_bodies)
        body->prepare_constraint_states();

    const std::size_t viters = world.joints.constraints.params.velocity_iterations;
    const std::size_t piters = world.joints.constraints.params.position_iterations;

    for (constraint2D *constraint : m_active_constraints)
        constraint->startup();

    for (std::size_t i = 0; i < viters; i++)
        for (constraint2D *constraint : m_active_constraints)
            constraint->solve_velocities();

    m_solved_positions = true;
    for (std::size_t i = 0; i < piters; i++)
    {
        m_solved_positions = true;
        for (constraint2D *constraint : m_active_constraints)
            m_solved_positions &= constraint->solve_positions();
        if (m_solved_positions)
            break;
    }
    for (contact2D *contact : m_active_contacts)
        contact->on_post_solve();

    if (world.rk_subset_index() != 0 || !world.islands.params.enable_sleep)
        return;

    m_energy = 0.f;
    for (body2D *body : m_bodies)
        m_energy += body->kinetic_energy();
    m_energy /= m_bodies.size();

    if (m_energy < world.islands.sleep_energy_threshold(this))
    {
        m_time_still += world.rk_substep_timestep();
        m_asleep = m_solved_positions && m_time_still >= world.islands.params.sleep_time_threshold;
    }
    else
        m_time_still = 0.f;
}

void island2D::collect_active_elements_and_call_pre_solve()
{
    m_active_actuators.clear();
    m_active_constraints.clear();
    m_active_contacts.clear();
    for (actuator2D *actuator : m_actuators)
        if (actuator->enabled()) [[likely]]
            m_active_actuators.push_back(actuator);
    for (constraint2D *constraint : m_constraints)
        if (constraint->enabled()) [[likely]]
            m_active_constraints.push_back(constraint);
    for (contact2D *contact : m_contacts)
        if (contact->enabled()) [[likely]]
        {
            contact->on_pre_solve();
            if (contact->enabled()) [[likely]]
                m_active_contacts.push_back(contact);
        }
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
