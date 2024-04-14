#pragma once

#include "ppx/joints/joint_container2D.hpp"

namespace ppx
{
template <Joint2D T> class joint_manager2D;

class ijoint_manager2D : public kit::identifiable<std::string>, public kit::yaml::codecable
{
  public:
    template <Joint2D T> using manager_t = joint_manager2D<T>;

    virtual ~ijoint_manager2D() = default;
    virtual void solve() = 0;
    virtual void on_body_removal_validation(const body2D *body) = 0;
    virtual bool remove(const joint2D *joint) = 0;

  protected:
    ijoint_manager2D(const std::string &name);
};

template <Joint2D T> class joint_manager2D : public joint_container2D<T>, public ijoint_manager2D
{
  public:
    virtual ~joint_manager2D() = default;

    joint_manager2D(world2D &world, const std::string &name) : joint_container2D<T>(world), ijoint_manager2D(name)
    {
        joint_container2D<T>::s_name = name;
    }

    using joint_container2D<T>::remove;
    bool remove(const joint2D *joint) override
    {
        return joint_container2D<T>::remove(joint);
    }

  private:
    void on_body_removal_validation(const body2D *body) override
    {
        joint_container2D<T>::on_body_removal_validation(body);
    }

    virtual void solve() override
    {
        for (T *joint : this->m_elements)
            joint->solve();
    }

#ifdef KIT_USE_YAML_CPP
    virtual YAML::Node encode() const override
    {
        return kit::yaml::codec<joint_container2D<T>>::encode(*this);
    }
    virtual bool decode(const YAML::Node &node) override
    {
        return kit::yaml::codec<joint_container2D<T>>::decode(node, *this);
    }
#endif
};

} // namespace ppx