#pragma once

#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/serialization/yaml/codec.hpp"

#ifdef _MSC_VER
#pragma warning(push) // Inheritance via dominance is intended
#pragma warning(disable : 4250)
#endif

namespace ppx
{
template <Constraint2D T> class constraint_manager2D;

class iconstraint_manager2D : virtual public ijoint_manager2D
{
  public:
    template <Constraint2D T> using manager_t = constraint_manager2D<T>;

    virtual ~iconstraint_manager2D() = default;

    virtual void startup() = 0;
    virtual void solve_velocities() = 0;
    virtual bool solve_positions() = 0;
};

template <Constraint2D T> class constraint_manager2D : public joint_manager2D<T>, public iconstraint_manager2D
{
  public:
    virtual ~constraint_manager2D() = default;

    constraint_manager2D(world2D &world, std::vector<joint2D *> &total_joints, manager_events<joint2D> &jevents,
                         const std::string &name)
        : kit::identifiable<std::string>(name), joint_manager2D<T>(world, total_joints, jevents)
    {
        joint_manager2D<T>::s_name = name;
    }

  private:
    virtual void startup() override
    {
        for (T *constraint : this->m_elements)
            if (constraint->enabled())
                constraint->startup();
    }

    virtual void solve_velocities() override
    {
        for (T *constraint : this->m_elements)
            if (constraint->enabled())
                constraint->solve_velocities();
    }

    virtual bool solve_positions() override
    {
        if constexpr (PVConstraint2D<T>)
        {
            bool solved = true;
            for (T *constraint : this->m_elements)
                if (constraint->enabled())
                    solved &= constraint->solve_positions();
            return solved;
        }
        else
            return true;
    };
};

} // namespace ppx

#ifdef _MSC_VER
#pragma warning(pop)
#endif