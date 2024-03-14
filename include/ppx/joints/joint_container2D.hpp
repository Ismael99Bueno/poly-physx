#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/joints/joint2D.hpp"
#include "kit/events/event.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/interface/indexable.hpp"

namespace ppx
{
template <typename T>
concept Joint = requires() {
    requires kit::DerivedFrom<T, joint2D>;
    typename T::ptr;
    typename T::const_ptr;
    typename T::specs;
};

template <Joint T> class joint_container2D : public manager2D<T>
{
  public:
    using ptr_t = typename T::ptr;
    using const_ptr_t = typename T::const_ptr;
    using specs = typename T::specs;

    virtual ~joint_container2D() = default;
    struct
    {
        kit::event<T &> on_addition;
        kit::event<const T &> on_early_removal;
        kit::event<std::size_t> on_late_removal;
    } events;

    virtual T &add(const specs &spc)
    {
        T &joint = this->m_elements.emplace_back(this->world, spc);
        joint.index = this->m_elements.size() - 1;
        KIT_ASSERT_ERROR(joint.valid(), "The joint must be valid to be able to add it into the simulation")
        events.on_addition(joint);
        return joint;
    }

    const_ptr_t ptr(std::size_t index) const
    {
        KIT_ASSERT_ERROR(index < this->m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                         this->m_elements.size())
        return {&this->m_elements, index};
    }
    ptr_t ptr(std::size_t index)
    {
        KIT_ASSERT_ERROR(index < this->m_elements.size(), "Index exceeds array bounds - index: {0}, size: {1}", index,
                         this->m_elements.size())
        return {&this->m_elements, index};
    }

    std::vector<const T *> from_body_ids(const kit::uuid id1, const kit::uuid id2) const
    {
        return from_body_ids<const T>(id1, id2, this->m_elements);
    }
    std::vector<T *> from_body_ids(const kit::uuid id1, const kit::uuid id2)
    {
        return from_body_ids<T>(id1, id2, this->m_elements);
    }

    using manager2D<T>::remove;
    virtual bool remove(std::size_t index) override
    {
        if (index >= this->m_elements.size())
            return false;

        events.on_early_removal(this->m_elements[index]);
        if (index != this->m_elements.size() - 1)
        {
            this->m_elements[index] = this->m_elements.back();
            this->m_elements[index].index = index;
        }
        this->m_elements.pop_back();
        events.on_late_removal(index);
        return true;
    }

    static const std::string &name()
    {
        return s_name;
    }

  protected:
    using manager2D<T>::manager2D;
    static inline std::string s_name;

    void validate()
    {
        std::size_t index = 0;
        for (auto it = this->m_elements.begin(); it != this->m_elements.end(); index++)
            if (!it->valid())
            {
                events.on_early_removal(*it);
                it = this->m_elements.erase(it);
                events.on_late_removal(index);
            }
            else
            {
                it->index = index;
                ++it;
            }
    }

  private:
    template <typename U, typename C>
    static std::vector<U *> from_body_ids(const kit::uuid id1, const kit::uuid id2, C &elements)
    {
        std::vector<U *> joints;
        for (U &joint : elements)
            if ((joint.body1()->id == id1 && joint.body2()->id == id2) ||
                (joint.body1()->id == id2 && joint.body2()->id == id1))
                joints.push_back(&joint);
        return joints;
    }
};

} // namespace ppx