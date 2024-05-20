#pragma once

#include "ppx/joints/joint_container2D.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/serialization/yaml/codec.hpp"

namespace ppx
{
template <VConstraint2D T> class constraint_manager2D;

class iconstraint_manager2D : public kit::identifiable<std::string>, public kit::toggleable, public kit::yaml::codecable
{
  public:
    template <VConstraint2D T> using manager_t = constraint_manager2D<T>;

    virtual ~iconstraint_manager2D() = default;

    virtual void startup() = 0;
    virtual void warmup() = 0;
    virtual void solve_velocities() = 0;
    virtual bool solve_positions() = 0;
    virtual void on_body_removal_validation(body2D *body) = 0;
    virtual bool remove(joint2D *constraint) = 0;

  protected:
    iconstraint_manager2D(const std::string &name);
};

template <VConstraint2D T> class constraint_manager2D : public joint_container2D<T>, public iconstraint_manager2D
{
  public:
    virtual ~constraint_manager2D() = default;

    constraint_manager2D(world2D &world, joint_events &jevents, const std::string &name)
        : joint_container2D<T>(world, jevents), iconstraint_manager2D(name)
    {
        joint_container2D<T>::s_name = name;
    }
    using joint_container2D<T>::remove;
    bool remove(joint2D *constraint) override
    {
        return joint_container2D<T>::remove(constraint);
    }

  private:
    void on_body_removal_validation(body2D *body) override
    {
        joint_container2D<T>::on_body_removal_validation(body);
    }

    virtual void startup() override
    {
        for (T *constraint : this->m_elements)
            if (constraint->enabled)
                constraint->startup();
    }

    virtual void warmup() override
    {
        for (T *constraint : this->m_elements)
            if (constraint->enabled)
                constraint->warmup();
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
            bool fully_adjusted = true;
            for (T *constraint : this->m_elements)
                if (constraint->enabled)
                    fully_adjusted &= constraint->solve_positions();
            return fully_adjusted;
        }
        else
            return true;
    };
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