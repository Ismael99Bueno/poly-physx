#pragma once

#include "ppx/constraints/constraint_meta_manager.hpp"
#include "ppx/actuators/actuator_meta_manager.hpp"

namespace ppx
{
class joint_repository2D final : public contiguous_manager2D<joint2D>
{
  public:
    actuator_meta_manager2D actuators;
    constraint_meta_manager2D constraints;

    template <Joint2D T> T *add(const typename T::specs &spc)
    {
        auto mng = manager<T>();
        KIT_ASSERT_ERROR(mng, "There is no manager of this type in the repository")
        return mng->add(spc);
    }

    template <Joint2D T> auto add_manager(const std::string &name)
    {
        if constexpr (Constraint2D<T>)
            return add_manager<T, constraint_manager2D<T>>(name);
        else
            return add_manager<T, actuator_manager2D<T>>(name);
    }

    template <Joint2D T, kit::DerivedFrom<actuator_manager2D<T>> Manager> Manager *add_manager(const std::string &name)
    {
        return actuators.add_manager<T, Manager>(name);
    }
    template <Joint2D T, kit::DerivedFrom<constraint_manager2D<T>> Manager>
    Manager *add_manager(const std::string &name)
    {
        return constraints.add_manager<T, Manager>(name);
    }

    template <Joint2D T> auto manager() const
    {
        if constexpr (Constraint2D<T>)
            return constraints.manager<T>();
        else
            return actuators.manager<T>();
    }
    template <Joint2D T> auto manager()
    {
        if constexpr (Constraint2D<T>)
            return constraints.manager<T>();
        else
            return actuators.manager<T>();
    }

    template <Joint2D T> bool remove_manager()
    {
        if constexpr (Constraint2D<T>)
            return constraints.remove<T>();
        else
            return actuators.remove<T>();
    }
    bool remove_manager(std::size_t index);

    bool remove(joint2D *joint) override final;
    bool remove(std::size_t index) override final;

    template <Joint2D T> bool remove(const std::size_t index)
    {
        auto mng = manager<T>();
        return mng ? mng->remove(index) : false;
    }
    template <Joint2D T> bool remove(T *joint)
    {
        auto mng = manager<T>();
        return mng ? mng->remove(joint) : false;
    }

    bool checksum() const;

  private:
    joint_repository2D(world2D &world);
    friend class world2D;
};
} // namespace ppx