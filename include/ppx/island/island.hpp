#pragma once

#include "ppx/actuators/actuator.hpp"
#include "ppx/constraints/constraint.hpp"
#include "ppx/body/body.hpp"
#include "ppx/collision/contacts/contact.hpp"
#include "kit/container/hashable_tuple.hpp"

namespace ppx
{
class island2D : public worldref2D
{
  public:
    using worldref2D::worldref2D;

    void awake();
    bool asleep() const;
    bool about_to_sleep() const;
    bool evaluate_split_candidate();

    void add_body(body2D *body);
    void remove_body(body2D *body);

    void merge(island2D &island);

    void solve_actuators(std::vector<state2D> &states);

    void solve_velocity_constraints(std::vector<state2D> &states);
    void solve_position_constraints(std::vector<state2D> &states);

    float time_still() const;
    float energy() const;
    bool solved_positions() const;

    const std::vector<body2D *> &bodies() const;
    const std::vector<actuator2D *> &actuators() const;
    const std::vector<constraint2D *> &constraints() const;

    bool checksum() const;

    template <IJoint2D Joint> static void add(Joint *joint)
    {
        KIT_PERF_SCOPE("ppx::island2D::add")
        island2D *island = get_effective_island(joint);
        if (!island)
            return;
        if constexpr (!Contact2D<Joint>)
            island->awake();
        else
        {
            KIT_ASSERT_ERROR(std::find(island->m_contacts.begin(), island->m_contacts.end(), joint) ==
                                 island->m_contacts.end(),
                             "Contact already exists on island")
            island->m_contacts.push_back(joint);
        }

        if constexpr (IConstraint2D<Joint>)
        {
            KIT_ASSERT_ERROR(std::find(island->m_constraints.begin(), island->m_constraints.end(), joint) ==
                                 island->m_constraints.end(),
                             "Joint already exists on island")
            island->m_constraints.push_back(joint);
        }
        else
        {
            KIT_ASSERT_ERROR(std::find(island->m_actuators.begin(), island->m_actuators.end(), joint) ==
                                 island->m_actuators.end(),
                             "Joint already exists on island")
            island->m_actuators.push_back(joint);
        }
    }

    template <IJoint2D Joint> static void remove(Joint *joint)
    {
        KIT_PERF_SCOPE("ppx::island2D::remove")
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
        if constexpr (Contact2D<Joint>)
        {
            island->m_lost_contact = true;
            for (std::size_t i = 0; i < island->m_contacts.size(); i++)
                if (island->m_contacts[i] == joint)
                {
                    island->m_contacts.erase(island->m_contacts.begin() + i);
                    break;
                }
        }

        if constexpr (IConstraint2D<Joint>)
        {
            for (std::size_t i = 0; i < island->m_constraints.size(); i++)
                if (island->m_constraints[i] == joint)
                {
                    island->m_constraints.erase(island->m_constraints.begin() + i);
                    if (!Contact2D<Joint> || !body2->is_dynamic() || island != body2->meta.island)
                        island->awake();
                    if constexpr (!Contact2D<Joint>)
                        island->m_may_split |= body2->is_dynamic();
                    return;
                }
        }
        else
            for (std::size_t i = 0; i < island->m_actuators.size(); i++)
                if (island->m_actuators[i] == joint)
                {
                    island->m_actuators.erase(island->m_actuators.begin() + i);
                    if (!Contact2D<Joint> || !body2->is_dynamic() || island != body2->meta.island)
                        island->awake();
                    if constexpr (!Contact2D<Joint>)
                        island->m_may_split |= body2->is_dynamic();
                    return;
                }
        KIT_WARN("Joint not found in island");
    }

    std::size_t size() const;
    bool is_void() const;
    bool no_bodies() const;
    bool no_joints() const;

  private:
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

    std::vector<body2D *> m_bodies;

    std::vector<actuator2D *> m_actuators;
    std::vector<constraint2D *> m_constraints;
    std::vector<contact2D *> m_contacts;

    std::uint32_t m_split_points = 0;
    float m_time_still = 0.f;
    float m_energy = 0.f;
    bool m_solved_positions = false;
    bool m_asleep = false;
    bool m_merged = false;
    bool m_may_split = false;
    bool m_lost_contact = false;

    friend class island_manager2D;
};
} // namespace ppx