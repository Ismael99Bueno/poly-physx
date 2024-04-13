#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/joints/joint2D.hpp"
#include "ppx/body/body2D.hpp"
#include "kit/events/event.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/block_allocator.hpp"

namespace ppx
{
template <Joint2D T> class joint_container2D : public manager2D<T>
{
  public:
    using specs = typename T::specs;

    virtual ~joint_container2D() = default;

    virtual T *add(const specs &spc)
    {
        T *joint = m_allocator.create(this->world, spc);
        joint->index = this->m_elements.size();
        this->m_elements.push_back(joint);
        joint->add_to_bodies();
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

        joint->remove_from_bodies();
        if (index != this->m_elements.size() - 1)
        {
            this->m_elements[index] = this->m_elements.back();
            this->m_elements[index]->index = index;
        }
        this->m_elements.pop_back();

        m_allocator.destroy(joint);
        return true;
    }
    bool remove(const joint2D *joint)
    {
        const T *tjoint = dynamic_cast<const T *>(joint);
        if (!tjoint)
            return false;
        return remove(tjoint);
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
        const auto &joints = body->joints();
        for (std::size_t i = joints.size() - 1; i < joints.size() && i >= 0; i--)
            remove(joints[i]->index);
    }

  private:
    template <typename U, typename C>
    static std::vector<U *> from_bodies(const body2D *body1, const body2D *body2, C &elements)
    {
        std::vector<U *> joints;
        for (U *joint : elements)
            if (joint->contains(body1) && joint->contains(body2))
                joints.push_back(joint);
        return joints;
    }
};

} // namespace ppx