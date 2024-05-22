#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/joints/joint2D.hpp"
#include "ppx/body/body2D.hpp"
#include "ppx/common/alias.hpp"
#include "ppx/joints/island2D.hpp"
#include "kit/events/event.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/interface/toggleable.hpp"

namespace ppx
{
class ijoint_manager2D : virtual public kit::identifiable<std::string>,
                         virtual public kit::toggleable,
                         virtual public kit::yaml::codecable
{
  public:
    virtual ~ijoint_manager2D() = default;

    virtual void on_body_removal_validation(body2D *body) = 0;
    virtual bool remove(joint2D *joint) = 0;
};

template <Joint2D T> class joint_manager2D : public manager2D<T>, virtual public ijoint_manager2D
{
  public:
    using specs = typename T::specs;

    virtual ~joint_manager2D() = default;

    virtual T *add(const specs &spc)
    {
        T *joint = allocator<T>::create(this->world, spc);
        joint->index = this->m_elements.size();
        this->m_elements.push_back(joint);
        m_total_joints.push_back(joint);

        joint->add_to_bodies();
        island2D::add(joint);

        this->events.on_addition(joint);
        m_jevents.on_addition(joint);
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
        m_jevents.on_removal(*joint);

        joint->remove_from_bodies();
        island2D::remove(joint);

        if (index != this->m_elements.size() - 1)
        {
            this->m_elements[index] = this->m_elements.back();
            this->m_elements[index]->index = index;
        }
        this->m_elements.pop_back();
        allocator<T>::destroy(joint);

        for (std::size_t i = 0; i < m_total_joints.size(); i++)
            if (m_total_joints[i] == joint)
            {
                m_total_joints.erase(m_total_joints.begin() + i);
                return true;
            }
        KIT_WARN("Joint not found in total joints");
        return true;
    }
    bool remove(joint2D *joint) override
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
    joint_manager2D(world2D &world, std::vector<joint2D *> &total_joints, manager_events<joint2D> &jevents)
        : manager2D<T>(world), m_total_joints(total_joints), m_jevents(jevents)
    {
    }

    std::vector<joint2D *> &m_total_joints;
    manager_events<joint2D> &m_jevents;

    static inline std::string s_name;

    void on_body_removal_validation(body2D *body) override
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

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override
    {
        return kit::yaml::codec<joint_manager2D<T>>::encode(*this);
    }
    virtual bool decode(const YAML::Node &node) override
    {
        return kit::yaml::codec<joint_manager2D<T>>::decode(node, *this);
    }
#endif
};

} // namespace ppx