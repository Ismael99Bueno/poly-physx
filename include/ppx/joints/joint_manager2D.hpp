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
    requires kit::Identifiable<T>;
    requires kit::Indexable<T>;
    typename T::ptr;
    typename T::const_ptr;
    typename T::specs;
};

class joint_solver2D : public kit::identifiable<>
{
  public:
    virtual ~joint_solver2D() = default;

  private:
    joint_solver2D(const kit::uuid &id);
    virtual void solve() = 0;
    virtual void validate() = 0;

    friend class joint_repository2D;
};

template <ValidJoint T> class joint_manager2D : public manager2D<T>, public joint_solver2D
{
  public:
    using ptr = typename T::ptr;
    using const_ptr = typename T::const_ptr;
    using specs = typename T::specs;

    virtual ~joint_manager2D() = default;
    struct
    {
        kit::event<T &> on_addition;
        kit::event<const T &> on_early_removal;
        kit::event<std::size_t> on_late_removal;
    } events;

    virtual T &add(const specs &spc = {})
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
    virtual bool remove(std::size_t index) override
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

  protected:
    joint_manager2D(world2D &world) : manager2D<T>(world), joint_solver2D(s_id)
    {
    }

  private:
    static inline kit::uuid s_id = kit::uuid::random();

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

    virtual void solve() override
    {
        if (m_elements.empty())
            return;

        if constexpr (std::is_base_of_v<constraint2D, T>)
        {
            if (world.constraints.warmup)
                for (T &joint)
                    joint.warmup();
            for (std::size_t i = 0; i < world.constraints.iterations; i++)
                for (T &joint : m_elements)
                    joint.solve();
        }
        else
            for (T &joint : m_elements)
                joint.solve();
    }
    friend class world2D;
};

class joint_repository2D final : public manager2D<kit::scope<joint_solver2D>>
{
  public:
    template <ValidJoint T, kit::DerivedFrom<joint_manager2D<T>> Manager = joint_manager2D<T>> Manager *add_manager()
    {
        KIT_ASSERT_ERROR(!contains(Manager::s_id), "There is already a solver of this type in the repository")
        auto manager = kit::make_scope<Manager>(world);
        T *ptr = manager.get();
        m_elements.push_back(std::move(manager));
        return ptr;
    }

    template <ValidJoint T, kit::DerivedFrom<joint_manager2D<T>> Manager = joint_manager2D<T>>
    const Manager *manager() const
    {
        const joint_solver2D *solver = (*this)[Manager::s_id];
        return solver ? static_cast<const Manager *>(solver) : nullptr;
    }
    template <ValidJoint T, kit::DerivedFrom<joint_manager2D<T>> Manager = joint_manager2D<T>> Manager *manager()
    {
        joint_solver2D *solver = (*this)[Manager::s_id];
        return solver ? static_cast<Manager *>(solver) : nullptr;
    }

    // wrappers around joint manager

    using manager2D<kit::scope<joint_solver2D>>::remove;
    bool remove(std::size_t index) override;
    template <ValidJoint T> bool remove()
    {
        for (std::size_t i = 0; i < m_elements.size(); i++)
            if (m_elements[i]->id == joint_manager2D<T>::s_id)
                return remove(i);
        return false;
    }

  private:
    joint_repository2D(world2D &world);
    void solve() const;
    void validate() const;

    friend class world2D;
};

} // namespace ppx