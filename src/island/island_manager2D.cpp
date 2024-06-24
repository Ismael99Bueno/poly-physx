#include "ppx/internal/pch.hpp"
#include "ppx/island/island_manager2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
void island_manager2D::solve()
{
#ifdef KIT_PROFILE
    KIT_ASSERT_WARN(!params.multithreaded, "Cannot run multiple threads if the KIT profiling tools are enabled")
    params.multithreaded = false;
#endif

    if (m_elements.empty())
        return;
    if (m_elements.size() == 1)
    {
        island2D *island = m_elements[0];
        if (!island->asleep())
            island->solve();
        return;
    }
    if (!params.multithreaded)
    {
        for (island2D *island : m_elements)
            if (!island->asleep())
                island->solve();
        return;
    }

    static std::vector<island2D *> awake_islands;
    awake_islands.clear();

    for (island2D *island : m_elements)
        if (!island->asleep())
            awake_islands.push_back(island);
    if (awake_islands.empty())
        return;

    if (awake_islands.size() == 1)
    {
        awake_islands[0]->solve();
        return;
    }
    kit::mt::for_each<PPX_THREAD_COUNT>(awake_islands,
                                        [](const std::size_t thread_idx, island2D *island) { island->solve(); });
}

void island_manager2D::remove_invalid()
{
    for (auto it = m_elements.begin(); it != m_elements.end();)
    {
        island2D *island = *it;
        if (island->merged || island->is_void())
        {
            allocator<island2D>::destroy(island);
            it = m_elements.erase(it);
        }
        else
            ++it;
    }
}

float island_manager2D::sleep_energy_threshold(const island2D *island) const
{
    const float lower = params.lower_sleep_energy_threshold;
    const float upper = params.upper_sleep_energy_threshold;
    const float bcount = (float)island->m_bodies.size();
    const float t = bcount / (bcount + (float)params.body_count_mid_threshold_reference);
    return lower + t * (upper - lower);
}

bool island_manager2D::enabled() const
{
    return m_enable;
}
void island_manager2D::enabled(bool enable)
{
    if (enable == m_enable)
        return;
    if (enable)
        build_from_existing_simulation();
    else
    {
        for (island2D *island : m_elements)
            allocator<island2D>::destroy(island);
        m_elements.clear();
        for (body2D *body : world.bodies)
            body->meta.island = nullptr;
    }
    m_enable = enable;
}

island2D *island_manager2D::create()
{
    island2D *island = allocator<island2D>::create(world);
    m_elements.push_back(island);
    return island;
}

island2D *island_manager2D::create_island_from_body(body2D *body)
{
    if (body->meta.island_flag || !body->is_dynamic())
        return nullptr;
    island2D *island = create();

    body->meta.island_flag = true;

    std::stack<body2D *> stack;
    stack.push(body);
    while (!stack.empty())
    {
        body2D *current = stack.top();
        stack.pop();
        island->add_body(current);

        const auto process_joint = [current, island, &stack](joint2D *joint) {
            if (joint->meta.island_flag)
                return;
            joint->meta.island_flag = true;

            if (joint->is_constraint())
                island->m_constraints.push_back(dynamic_cast<constraint2D *>(joint));
            else
                island->m_actuators.push_back(dynamic_cast<actuator2D *>(joint));

            body2D *other = joint->other(current);
            if (!other->meta.island_flag && other->is_dynamic())
            {
                other->meta.island_flag = true;
                stack.push(other);
            }
        };
        for (joint2D *joint : current->meta.joints)
            process_joint(joint);
        for (contact2D *contact : current->meta.contacts)
        {
            joint2D *joint = dynamic_cast<joint2D *>(contact);
            KIT_ASSERT_ERROR(joint, "Contact is not a joint");
            if (!joint->meta.island_flag)
                island->m_contacts.push_back(contact);
            process_joint(joint);
        }
    }
    return island;
}

bool island_manager2D::remove(const std::size_t index)
{
    if (index >= m_elements.size())
        return false;
    island2D *island = m_elements[index];
    allocator<island2D>::destroy(island);
    m_elements.erase(m_elements.begin() + index);
    return true;
}

void island_manager2D::build_from_existing_simulation()
{
    for (body2D *body : world.bodies)
        body->meta.island_flag = false;
    for (joint2D *joint : world.joints)
        joint->meta.island_flag = false;
    for (contact2D *contact : world.collisions.contact_solver()->create_total_contacts_list())
    {
        joint2D *joint = dynamic_cast<joint2D *>(contact);
        KIT_ASSERT_ERROR(joint, "Contact is not a joint");
        joint->meta.island_flag = false;
    }

    for (body2D *body : world.bodies)
        create_island_from_body(body);
}

void island_manager2D::try_split(std::uint32_t max_splits)
{
    if (m_elements.empty())
        return;
    std::size_t iters = 0;
    const std::size_t size = m_elements.size();
    m_island_to_split = glm::min(m_island_to_split, size - 1);
    while (iters++ < size && max_splits > 0)
    {
        island2D *island = m_elements[m_island_to_split];
        if (island->can_split() && split(island))
            max_splits--;
        if (m_island_to_split-- == 0)
            m_island_to_split = m_elements.size() - 1;
    }
}

bool island_manager2D::split(island2D *island)
{
    const bool was_asleep = island->asleep();
    const float time_still = island->time_still();

    for (body2D *body : island->m_bodies)
    {
        KIT_ASSERT_ERROR(body->is_dynamic(), "Body must be dynamic");
        body->meta.island_flag = false;
    }
    for (joint2D *joint : island->m_actuators)
        joint->meta.island_flag = false;
    for (constraint2D *constraint : island->m_constraints)
        constraint->meta.island_flag = false;
    std::uint32_t split_count = 0;

    for (body2D *body : island->m_bodies)
    {
        island2D *new_island = create_island_from_body(body);
        if (!new_island)
            continue;
        new_island->m_asleep = was_asleep;
        new_island->m_time_still = time_still;
        split_count++;
        if (new_island->m_bodies.size() == island->m_bodies.size())
            break;
    }

    allocator<island2D>::destroy(island);
    m_elements.erase(m_elements.begin() + m_island_to_split);
    return split_count > 1;
}

bool island_manager2D::checksum() const
{
    std::size_t dynamic_bodies = 0;
    for (body2D *body : world.bodies)
        if (body->is_dynamic())
            dynamic_bodies++;

    const std::size_t contacts = world.collisions.contact_solver()->total_contacts_count();
    const std::size_t joints = world.joints.size();

    std::size_t body_count = 0;
    std::size_t contact_count = 0;
    std::size_t joint_count = 0;

    for (const island2D *island : m_elements)
    {
        if (!island->checksum())
            return false;
        if (island->no_bodies() && !island->no_joints())
        {
            KIT_ERROR("Cannot have an island that has no bodies but does have joints")
            return false;
        }
        body_count += island->m_bodies.size();
        for (actuator2D *actuator : island->m_actuators)
            if (dynamic_cast<contact2D *>(actuator))
                contact_count++;
            else
                joint_count++;
        for (constraint2D *constraint : island->m_constraints)
            if (dynamic_cast<contact2D *>(constraint))
                contact_count++;
            else
                joint_count++;
    }

    const std::unordered_set<const island2D *> islands(m_elements.begin(), m_elements.end());
    KIT_ASSERT_ERROR(body_count == dynamic_bodies, "Island checksum failed: Body count mismatch")
    KIT_ASSERT_ERROR(contact_count == contacts, "Island checksum failed: Contact count mismatch")
    KIT_ASSERT_ERROR(joint_count == joints, "Island checksum failed: Joint count mismatch")
    KIT_ASSERT_ERROR(islands.size() == m_elements.size(), "Island checksum failed: Duplicate islands found")
    return body_count == dynamic_bodies && contact_count == contacts && joint_count == joints &&
           islands.size() == m_elements.size();
}

} // namespace ppx