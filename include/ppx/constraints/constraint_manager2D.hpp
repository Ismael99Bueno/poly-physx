#pragma once

#include "ppx/joints/joint_container2D.hpp"
#include "ppx/constraints/pvconstraint2D.hpp"
#include "kit/serialization/yaml/codec.hpp"

namespace ppx
{
template <typename T>
concept Constraint = requires() {
    requires Joint<T>;
    requires kit::DerivedFrom<T, vconstraint2D>;
};

template <Constraint T> class constraint_manager2D;

class iconstraint_manager2D : public kit::identifiable<std::string>, public kit::yaml::codecable
{
  public:
    template <Constraint T> using manager_t = constraint_manager2D<T>;

    virtual ~iconstraint_manager2D() = default;

    virtual void startup() = 0;
    virtual void solve() = 0;
    virtual bool adjust_positions() = 0;
    virtual void on_body_removal_validation(const body2D *body) = 0;

  protected:
    iconstraint_manager2D(const std::string &name);
};

template <Constraint T> class constraint_manager2D : public joint_container2D<T>, public iconstraint_manager2D
{
  public:
    virtual ~constraint_manager2D() = default;

    constraint_manager2D(world2D &world, const std::string &name)
        : joint_container2D<T>(world), iconstraint_manager2D(name)
    {
        joint_container2D<T>::s_name = name;
    }

  private:
    void on_body_removal_validation(const body2D *body) override
    {
        joint_container2D<T>::on_body_removal_validation(body);
    }

    virtual void startup() override
    {
        for (T *constraint : this->m_elements)
        {
            constraint->startup();
            if (this->world.constraints.warmup)
                constraint->warmup();
        }
    }

    virtual void solve() override
    {
        for (T *constraint : this->m_elements)
            constraint->solve();
    }

    virtual bool adjust_positions() override
    {
        if constexpr (std::is_base_of_v<pvconstraint2D, T>)
        {
            bool fully_adjusted = true;
            for (T *constraint : this->m_elements)
                fully_adjusted &= constraint->adjust_positions();
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