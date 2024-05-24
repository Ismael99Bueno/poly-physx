#pragma once

#include "ppx/actuators/actuator2D.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/body/body2D.hpp"

namespace ppx
{
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

    const std::vector<body2D *> &bodies() const;
    const std::vector<actuator2D *> &actuators() const;
    const std::vector<constraint2D *> &constraints() const;

    template <IJoint2D Joint> static void add(Joint *joint)
    {
        island2D *island = get_effective_island(joint);
        if (!island)
            return;

        island->awake();
        if constexpr (IConstraint2D<Joint>)
        {
            if (std::find(island->m_constraints.begin(), island->m_constraints.end(), joint) ==
                island->m_constraints.end())
                island->m_constraints.push_back(joint);
        }
        else
        {
            if (std::find(island->m_actuators.begin(), island->m_actuators.end(), joint) == island->m_actuators.end())
                island->m_actuators.push_back(joint);
        }
    }

    template <IJoint2D Joint> static void remove(Joint *joint)
    {
        const body2D *body1 = joint->body1();
        const body2D *body2 = joint->body2();
        KIT_ASSERT_ERROR(body1->is_dynamic() || body2->is_dynamic(), "At least one body must be dynamic");
        if (!body1->is_dynamic())
            std::swap(body1, body2);

        island2D *island = body1->meta.island;
        KIT_ASSERT_ERROR(island == body2->meta.island || !body2->is_dynamic(),
                         "The joint's bodies must share the same island");
        if (!island)
            return;
        if constexpr (IConstraint2D<Joint>)
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
            for (std::size_t i = 0; i < island->m_actuators.size(); i++)
                if (island->m_actuators[i] == joint)
                {
                    island->m_actuators.erase(island->m_actuators.begin() + i);
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

    std::vector<actuator2D *> m_actuators;
    std::vector<constraint2D *> m_constraints;

    template <IJoint2D Joint> static island2D *get_effective_island(Joint *joint)
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

    friend class island_manager2D;
};
} // namespace ppx