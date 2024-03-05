#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/joints/joint2D.hpp"
#include "kit/events/event.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/interface/indexable.hpp"

namespace ppx
{
template <typename T>
concept ValidJoint = requires(T t) {
    requires kit::DerivedFrom<T, joint2D>;
    requires kit::Indexable<T>;
    typename T::ptr;
    typename T::const_ptr;
    typename T::specs;
};

class joint_solver2D
{
  public:
    virtual ~joint_solver2D() = default;

  private:
    virtual void solve() = 0;
    virtual void validate() = 0;

    friend class joint_repository2D;
};

template <ValidJoint T> class joint_manager2D final : public manager2D<T>, public joint_solver2D
{
  public:
    using ptr = typename T::ptr;
    using const_ptr = typename T::const_ptr;
    using specs = typename T::specs;

    struct
    {
        kit::event<T &> on_addition;
        kit::event<const T &> on_early_removal;
        kit::event<std::size_t> on_late_removal;
    } events;

    T &add(const specs &spc = {})
    {
        T &joint = m_elements.emplace_back(world, spc);
        joint.index = m_elements.size() - 1;
        KIT_ASSERT_ERROR(joint.valid(), "The joint must be valid to be able to add it into the simulation")
        events.on_addition(joint);
        return joint;
    }

    std::vector<const T *> from_ids(kit::uuid id1, kit::uuid id2) const;
    std::vector<T *> from_ids(kit::uuid id1, kit::uuid id2);

    const_ptr ptr(std::size_t index) const
    {
        KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                         m_elements.size())
        return {&m_elements, index};
    }
    ptr ptr(std::size_t index)
    {
        KIT_ASSERT_ERROR(index < m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                         m_elements.size())
        return {&m_elements, index};
    }

    using manager2D<T>::remove;
    bool remove(std::size_t index)
    {
        if (index >= m_elements.size())
            return false;

        events.on_early_removal(m_elements[index]);
        if (index != m_elements.size() - 1)
        {
            m_elements[index] = m_elements.back();
            m_elements[index].index = index;
        }
        m_elements.pop_back();
        events.on_late_removal(index);
        return true;
    }

  private:
    using manager2D<T>::manager2D;
    static inline std::size_t s_index = SIZE_MAX;

    void validate() override
    {
        std::size_t index = 0;
        for (auto it = m_elements.begin(); it != m_elements.end(); index++)
            if (!it->joint.valid())
            {
                events.on_early_removal(*it);
                it = m_elements.erase(it);
                events.on_late_removal(index);
            }
            else
            {
                it->index = index;
                ++it;
            }
    }

    void solve() override
    {
        if (m_elements.empty())
            return;
    }
    friend class world2D;
};

class joint_repository2D
{
  public:
    template <ValidJoint T, class... JointArgs> T *add(world2D &world, JointArgs &&...args)
    {
        KIT_ASSERT_ERROR(joint_manager2D<T>::s_index == SIZE_MAX,
                         "There is already a solver of this type in the repository")
        joint_manager2D<T>::s_index = m_solvers.size();
        auto joint = kit::make_scope<joint_manager2D<T>>(world, std::forward<JointArgs>(args)...);
        T *ptr = joint.get();
        m_solvers.push_back(std::move(joint));
        return ptr;
    }

  private:
    std::vector<kit::scope<joint2D>> m_solvers;
};

} // namespace ppx