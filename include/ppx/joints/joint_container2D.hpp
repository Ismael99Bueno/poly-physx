#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/joints/joint2D.hpp"
#include "kit/events/event.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/block_allocator.hpp"

namespace ppx
{
template <typename T>
concept Joint = requires() {
    requires kit::DerivedFrom<T, joint2D>;
    typename T::specs;
};

template <Joint T> class joint_container2D : public manager2D<T>
{
  public:
    using specs = typename T::specs;

    virtual ~joint_container2D() = default;

    virtual T *add(const specs &spc)
    {
        T *joint = m_allocator.create(this->world, spc);
        joint->index = this->m_elements.size();
        this->m_elements.push_back(joint);
        this->events.on_addition(joint);
        return joint;
    }

    std::vector<const T *> from_bodies(const body2D *body1, const body2D *body2) const
    {
        return from_bodies<const T>(body1, body2, this->m_elements);
    }
    std::vector<T *> from_bodies(const body2D *body1, const body2D *body2)
    {
        return from_bodies<T>(body1, body2, this->m_elements);
    }

    using manager2D<T>::remove;
    virtual bool remove(const std::size_t index) override
    {
        if (index >= this->m_elements.size())
            return false;

        T *joint = this->m_elements[index];
        this->events.on_removal(*joint);

        if (index != this->m_elements.size() - 1)
        {
            this->m_elements[index] = this->m_elements.back();
            this->m_elements[index]->index = index;
        }
        this->m_elements.pop_back();

        m_allocator.destroy(joint);
        return true;
    }

    static const std::string &name()
    {
        return s_name;
    }

  protected:
    using manager2D<T>::manager2D;
    kit::block_allocator<T> m_allocator;

    static inline std::string s_name;

    void on_body_removal_validation(const body2D *body)
    {
        for (std::size_t i = this->m_elements.size() - 1; i < this->m_elements.size() && i >= 0; i--)
            if (this->m_elements[i]->body1() == body || this->m_elements[i]->body2() == body)
                remove(i);
    }

  private:
    template <typename U, typename C>
    static std::vector<U *> from_bodies(const body2D *body1, const body2D *body2, C &elements)
    {
        std::vector<U *> joints;
        for (U *joint : elements)
            if ((joint->body1() == body1 && joint->body2() == body2) ||
                (joint->body1() == body2 && joint->body2() == body1))
                joints.push_back(joint);
        return joints;
    }
};

} // namespace ppx