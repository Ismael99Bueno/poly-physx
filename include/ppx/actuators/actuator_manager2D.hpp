#pragma once

#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/actuators/actuator2D.hpp"
#include "kit/serialization/yaml/codec.hpp"

#ifdef _MSC_VER
#pragma warning(push) // Inheritance via dominance is intended
#pragma warning(disable : 4250)
#endif

namespace ppx
{
template <Actuator2D T> class actuator_manager2D;

class iactuator_manager2D : virtual public ijoint_manager2D
{
  public:
    template <Actuator2D T> using manager_t = actuator_manager2D<T>;

    virtual ~iactuator_manager2D() = default;
    virtual void solve() = 0;
};

template <Actuator2D T> class actuator_manager2D : public joint_manager2D<T>, public iactuator_manager2D
{
  public:
    virtual ~actuator_manager2D() = default;

    actuator_manager2D(world2D &world, std::vector<joint2D *> &total_joints, manager_events<joint2D> &jevents,
                       const std::string &name)
        : kit::identifiable<std::string>(name), joint_manager2D<T>(world, total_joints, jevents)
    {
        joint_manager2D<T>::s_name = name;
    }

  private:
    virtual void solve() override
    {
        for (T *actuator : this->m_elements)
            if (actuator->enabled())
                actuator->solve();
    }
};

} // namespace ppx

#ifdef _MSC_VER
#pragma warning(pop)
#endif