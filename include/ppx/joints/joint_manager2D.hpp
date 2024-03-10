#pragma once

#include "ppx/joints/joint_container2D.hpp"
#include "ppx/joints/meta_manager2D.hpp"

namespace ppx
{
template <Joint T> class joint_manager2D;

class joint_solver2D : public kit::identifiable<std::string>, public kit::yaml::codecable
{
  public:
    template <Joint T> using manager_t = joint_manager2D<T>;

    virtual ~joint_solver2D() = default;

  protected:
    joint_solver2D(const std::string &name);

  private:
    virtual void solve() = 0;
    virtual void validate() = 0;

    friend class joint_meta_manager2D;
};

template <Joint T> class joint_manager2D : public joint_container2D<T>, public joint_solver2D
{
  public:
    virtual ~joint_manager2D() = default;

    joint_manager2D(world2D &world, const std::string &name) : joint_container2D<T>(world), joint_solver2D(name)
    {
        joint_container2D<T>::s_name = name;
    }

  private:
    void validate() override
    {
        joint_container2D<T>::validate();
    }

    virtual void solve() override
    {
        for (T &joint : this->m_elements)
            joint.solve();
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

class joint_meta_manager2D final : public meta_manager2D<joint_solver2D>
{
    using meta_manager2D<joint_solver2D>::meta_manager2D;

    void solve();
    void validate();

    friend class world2D;
    friend class joint_repository2D;
};

} // namespace ppx