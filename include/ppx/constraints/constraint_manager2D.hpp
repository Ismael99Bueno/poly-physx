#pragma once

#include "ppx/joints/joint_container2D.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/serialization/yaml/codec.hpp"

namespace ppx
{
template <VConstraint2D T> class constraint_manager2D;

class iconstraint_manager2D : public virtual ijoint_container2D
{
  public:
    template <VConstraint2D T> using manager_t = constraint_manager2D<T>;

    virtual ~iconstraint_manager2D() = default;

    virtual void startup() = 0;
    virtual void solve_velocities() = 0;
    virtual bool solve_positions() = 0;
};

template <VConstraint2D T> class constraint_manager2D : public joint_container2D<T>, public iconstraint_manager2D
{
  public:
    virtual ~constraint_manager2D() = default;

    constraint_manager2D(world2D &world, std::vector<joint2D *> &total_joints, manager_events<joint2D> &jevents,
                         const std::string &name)
        : kit::identifiable<std::string>(name), joint_container2D<T>(world, total_joints, jevents)
    {
        joint_container2D<T>::s_name = name;
    }

  private:
    virtual void startup() override
    {
        for (T *constraint : this->m_elements)
            if (constraint->enabled)
                constraint->startup();
    }

    virtual void solve_velocities() override
    {
        for (T *constraint : this->m_elements)
            if (constraint->enabled)
                constraint->solve_velocities();
    }

    virtual bool solve_positions() override
    {
        if constexpr (PVConstraint2D<T>)
        {
            bool solved = true;
            for (T *constraint : this->m_elements)
                if (constraint->enabled)
                    solved &= constraint->solve_positions();
            return solved;
        }
        else
            return true;
    };
};

} // namespace ppx