#pragma once

#include "ppx/collision/contacts/contact_joint2D.hpp"
#include "ppx/collision/contacts/contact_constraint2D.hpp"

namespace ppx
{
template <typename T>
concept ValidJoint = kit::DerivedFrom<T, joint2D>;

class island2D : public worldref2D
{
  public:
    using worldref2D::worldref2D;

    bool may_split = false;
    bool merged = false;

    void awake();
    bool asleep() const;

    void add_body(body2D *body);
    void remove_body(body2D *body);

    void merge(island2D &island);

    void prepare_constraint_states();
    void solve();

    template <ValidJoint VJ> static void add(VJ *joint)
    {
        island2D *island = get_effective_island(joint);
        if (!island)
            return;

        island->awake();
        if constexpr (kit::DerivedFrom<VJ, constraint2D>)
        {
            if (std::find(island->m_constraints.begin(), island->m_constraints.end(), joint) ==
                island->m_constraints.end())
                island->m_constraints.push_back(joint);
        }
        else
        {
            if (std::find(island->m_joints.begin(), island->m_joints.end(), joint) == island->m_joints.end())
                island->m_joints.push_back(joint);
        }
    }

    template <ValidJoint VJ> static void remove(VJ *joint)
    {
        const body2D *body1 = joint->body1();
        const body2D *body2 = joint->body2();
        KIT_ASSERT_ERROR(body1->is_dynamic() || body2->is_dynamic(), "At least one body must be dynamic");
        if (!body1->is_dynamic())
            std::swap(body1, body2);

        island2D *island = body1->meta.island;
        KIT_ASSERT_ERROR(island == joint->body2()->meta.island || !body2->is_dynamic(),
                         "The joint's bodies must share the same island");
        if (!island)
            return;
        if constexpr (kit::DerivedFrom<VJ, constraint2D>)
        {
            for (std::size_t i = 0; i < island->m_constraints.size(); i++)
                if (island->m_constraints[i] == joint)
                {
                    island->m_constraints.erase(island->m_constraints.begin() + i);
                    island->awake();
                    island->may_split = body1->is_dynamic() && body2->is_dynamic();
                    return;
                }
        }
        else
            for (std::size_t i = 0; i < island->m_joints.size(); i++)
                if (island->m_joints[i] == joint)
                {
                    island->m_joints.erase(island->m_joints.begin() + i);
                    island->awake();
                    island->may_split = body1->is_dynamic() && body2->is_dynamic();
                    return;
                }
        KIT_WARN("Joint not found in island");
    }

    std::size_t size() const;
    bool empty() const;

  private:
    std::vector<body2D *> m_bodies;

    std::vector<joint2D *> m_joints;
    std::vector<constraint2D *> m_constraints;

    template <ValidJoint VJ> static island2D *get_effective_island(VJ *joint)
    {
        const body2D *body1 = joint->body1();
        const body2D *body2 = joint->body2();
        KIT_ASSERT_ERROR(body1->is_dynamic() || body2->is_dynamic(), "At least one body must be dynamic");
        if (!body1->is_dynamic())
            std::swap(body1, body2);

        island2D *island1 = body1->meta.island;
        if (!island1)
            return nullptr;
        island2D *island2 = body2->meta.island;
        KIT_ASSERT_ERROR(island2 || !body2->is_dynamic(),
                         "Found a nullptr and non nullptr island in the same joint/contact");
        if (!island2)
            return island1;
        return handle_island_merge_encounter(island1, island2);
    }

    static island2D *handle_island_merge_encounter(island2D *island1, island2D *island2);

    float m_time_still = 0.f;
    bool m_asleep = false;

    friend class joint_repository2D;
};
} // namespace ppx