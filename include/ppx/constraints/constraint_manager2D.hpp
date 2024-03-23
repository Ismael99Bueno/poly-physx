#pragma once

#include "ppx/joints/joint_container2D.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/joints/meta_manager2D.hpp"
#include "kit/serialization/yaml/codec.hpp"

namespace ppx
{
template <typename T>
concept Constraint = requires() {
    requires Joint<T>;
    requires kit::DerivedFrom<T, constraint2D>;
};

template <Constraint T> class constraint_manager2D;

class constraint_solver2D : public kit::identifiable<std::string>, public kit::yaml::codecable
{
  public:
    template <Constraint T> using manager_t = constraint_manager2D<T>;

    virtual ~constraint_solver2D() = default;

  protected:
    constraint_solver2D(const std::string &name);

  private:
    virtual void startup() = 0;
    virtual void solve() = 0;
    virtual void on_body_removal_validation() = 0;

    friend class constraint_meta_manager2D;
};

template <Constraint T> class constraint_manager2D : public joint_container2D<T>, public constraint_solver2D
{
  public:
    virtual ~constraint_manager2D() = default;

    constraint_manager2D(world2D &world, const std::string &name)
        : joint_container2D<T>(world), constraint_solver2D(name)
    {
        joint_container2D<T>::s_name = name;
    }

  private:
    void on_body_removal_validation() override
    {
        joint_container2D<T>::on_body_removal_validation();
    }

    virtual void startup() override
    {
        for (T &constraint : this->m_elements)
        {
            constraint.startup();
            if (this->world.constraints.warmup)
                constraint.warmup();
        }
    }

    virtual void solve() override
    {
        for (std::size_t i = 0; i < this->world.constraints.iterations; i++)
            for (T &constraint : this->m_elements)
                constraint.solve();
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

class constraint_meta_manager2D final : public meta_manager2D<constraint_solver2D>
{
    using meta_manager2D<constraint_solver2D>::meta_manager2D;

    void startup();
    void solve();
    void on_body_removal_validation();

    friend class world2D;
    friend class joint_repository2D;
};

} // namespace ppx