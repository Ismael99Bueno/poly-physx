#include "ppx/internal/pch.hpp"
#include "ppx/joints/joint_repository2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
joint_repository2D::joint_repository2D(world2D &world)
    : manager2D(world), non_constraint_based(world, m_elements, events), constraint_based(world, m_elements, events)
{
}

void joint_repository2D::solve_islands() // PARALLELIZE THIS
{
    for (auto it = m_islands.begin(); it != m_islands.end(); ++it)
    {
        island2D *island = *it;
        if (island->merged || island->empty())
        {
            allocator<island2D>::destroy(island);
            it = m_islands.erase(it);
            if (it == m_islands.end())
                return;
            island = *it;
        }
        if (!island->asleep())
            island->solve();
    }
}

bool joint_repository2D::islands_enabled() const
{
    return m_enable_islands;
}
void joint_repository2D::islands_enabled(bool enable)
{
    if (enable == m_enable_islands)
        return;
    if (enable)
        build_islands_from_existing_simulation();
    else
    {
        for (island2D *island : m_islands)
            allocator<island2D>::destroy(island);
        m_islands.clear();
        for (body2D *body : world.bodies)
            body->meta.island = nullptr;
    }
    m_enable_islands = enable;
}

island2D *joint_repository2D::create_island()
{
    island2D *island = allocator<island2D>::create(world);
    m_islands.push_back(island);
    return island;
}

void joint_repository2D::build_islands_from_existing_simulation()
{
    for (body2D *body : world.bodies)
        body->meta.island_flag = false;
    for (joint2D *joint : world.joints)
        joint->meta.island_flag = false;

    std::stack<body2D *> stack;
    for (body2D *body : world.bodies)
    {
        if (body->meta.island_flag)
            continue;
        island2D *island = create_island();
        stack.push(body);
        while (!stack.empty())
        {
            body2D *current = stack.top();
            stack.pop();
            if (current->meta.island_flag)
                continue;
            current->meta.island_flag = true;
            island->add_body(current);

            const auto process_joint = [current, island, &stack](joint2D *joint) {
                if (joint->meta.island_flag)
                    return;

                joint->meta.island_flag = true;
                if (joint->is_constraint())
                    island->m_constraints.push_back((constraint2D *)joint);
                else
                    island->m_joints.push_back(joint);
                body2D *other = joint->body1() == current ? joint->body2() : joint->body1();
                if (!other->meta.island_flag && other->is_dynamic())
                    stack.push(other);
            };
            for (joint2D *joint : current->meta.joints)
                process_joint(joint);
            for (contact2D *contact : current->meta.contacts)
            {
                joint2D *joint = dynamic_cast<joint2D *>(contact);
                KIT_ASSERT_ERROR(joint, "Contact is not a joint");
                process_joint(joint);
            }
        }
    }
}

void joint_repository2D::try_split_islands(std::uint32_t max_splits)
{
    if (m_islands.empty())
        return;
    std::size_t iters = 0;
    const std::size_t size = m_islands.size();
    m_island_to_split = glm::min(m_island_to_split, size - 1);
    while (iters++ < size && max_splits > 0)
    {
        island2D *island = m_islands[m_island_to_split];
        if (island->may_split && !island->merged && !island->empty() && split_island(island))
            max_splits--;
        if (m_island_to_split-- == 0)
            m_island_to_split = m_islands.size() - 1;
    }
}

bool joint_repository2D::split_island(island2D *island)
{
    for (body2D *body : island->m_bodies)
    {
        KIT_ASSERT_ERROR(body->is_dynamic(), "Body must be dynamic");
        body->meta.island_flag = false;
    }
    for (joint2D *joint : island->m_joints)
        joint->meta.island_flag = false;
    for (constraint2D *constraint : island->m_constraints)
        constraint->meta.island_flag = false;
    std::uint32_t split_count = 0;

    std::stack<body2D *> stack;
    for (body2D *body : island->m_bodies)
    {
        if (body->meta.island_flag)
            continue;
        island2D *new_island = create_island();
        split_count++;
        stack.push(body);
        while (!stack.empty())
        {
            body2D *current = stack.top();
            stack.pop();
            current->meta.island_flag = true;
            new_island->add_body(current);

            const auto process_joint = [current, new_island, &stack](joint2D *joint) {
                if (joint->meta.island_flag)
                    return;
                joint->meta.island_flag = true;

                if (joint->is_constraint())
                    new_island->m_constraints.push_back((constraint2D *)joint);
                else
                    new_island->m_joints.push_back(joint);
                body2D *other = joint->body1() == current ? joint->body2() : joint->body1();
                if (!other->meta.island_flag && other->is_dynamic())
                    stack.push(other);
            };
            for (joint2D *joint : current->meta.joints)
                process_joint(joint);
            for (contact2D *contact : current->meta.contacts)
            {
                joint2D *joint = dynamic_cast<joint2D *>(contact);
                KIT_ASSERT_ERROR(joint, "Contact is not a joint");
                process_joint(joint);
            }
        }
    }

    allocator<island2D>::destroy(island);
    KIT_ASSERT_ERROR(m_islands[m_island_to_split] == island, "Island mismatch")
    m_islands.erase(m_islands.begin() + m_island_to_split);
    return split_count > 1;
}

bool joint_repository2D::remove(std::size_t index)
{
    if (non_constraint_based.remove(index))
        return true;
    return constraint_based.remove(index);
}

} // namespace ppx