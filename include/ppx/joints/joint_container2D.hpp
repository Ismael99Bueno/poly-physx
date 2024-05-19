#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/joints/joint2D.hpp"
#include "ppx/body/body2D.hpp"
#include "ppx/common/alias.hpp"
#include "kit/events/event.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/interface/indexable.hpp"

namespace ppx
{

struct joint_events
{
    kit::event<joint2D *> on_addition;
    kit::event<joint2D &> on_removal;
};

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
        jevents.on_addition(joint);
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
        jevents.on_removal(*joint);

        joint->remove_from_bodies();
        joint->body1()->awake(true);
        joint->body2()->awake(true);
        if (index != this->m_elements.size() - 1)
        {
            this->m_elements[index] = this->m_elements.back();
            this->m_elements[index]->index = index;
        }
        this->m_elements.pop_back();

        m_allocator.destroy(joint);
        return true;
    }
    bool remove(joint2D *joint)
    {
        T *tjoint = dynamic_cast<T *>(joint);
        if (!tjoint)
            return false;
        return remove(tjoint);
    }

    static const std::string &name()
    {
        return s_name;
    }

  protected:
    joint_container2D(world2D &world, joint_events &jevents) : manager2D<T>(world), jevents(jevents)
    {
    }
    allocator<T> m_allocator;
    joint_events &jevents;

    static inline std::string s_name;

    void on_body_removal_validation(body2D *body)
    {
        const auto &joints = body->meta.joints;
        for (std::size_t i = joints.size() - 1; i < joints.size() && i >= 0; i--)
            remove(joints[i]);
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