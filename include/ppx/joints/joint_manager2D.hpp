#pragma once

#include "ppx/joints/joint_container2D.hpp"
#include "kit/serialization/yaml/codec.hpp"

namespace ppx
{
template <Joint2D T> class joint_manager2D;

class ijoint_manager2D : public virtual ijoint_container2D
{
  public:
    template <Joint2D T> using manager_t = joint_manager2D<T>;

    virtual ~ijoint_manager2D() = default;
    virtual void solve() = 0;
};

template <Joint2D T> class joint_manager2D : public joint_container2D<T>, public ijoint_manager2D
{
  public:
    virtual ~joint_manager2D() = default;

    joint_manager2D(world2D &world, std::vector<joint2D *> &total_joints, manager_events<joint2D> &jevents,
                    const std::string &name)
        : kit::identifiable<std::string>(name), joint_container2D<T>(world, total_joints, jevents)
    {
        joint_container2D<T>::s_name = name;
    }

  private:
    virtual void solve() override
    {
        for (T *joint : this->m_elements)
            if (joint->enabled)
                joint->solve();
    }
};

} // namespace ppx