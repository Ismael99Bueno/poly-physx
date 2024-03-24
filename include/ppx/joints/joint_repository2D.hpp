#pragma once

#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"

namespace ppx
{
class joint_repository2D
{
  public:
    joint_meta_manager2D non_constraint_based;
    constraint_meta_manager2D constraint_based;

    template <Joint T> T *add(const typename T::specs &spc)
    {
        if constexpr (std::is_base_of_v<constraint2D, T>)
            return constraint_based.add<T>(spc);
        else
            return non_constraint_based.add<T>(spc);
    }

    template <Joint T> auto add_manager(const std::string &name)
    {
        if constexpr (std::is_base_of_v<constraint2D, T>)
            return add_manager<T, constraint_manager2D<T>>(name);
        else
            return add_manager<T, joint_manager2D<T>>(name);
    }

    template <Joint T, kit::DerivedFrom<joint_manager2D<T>> Manager> Manager *add_manager(const std::string &name)
    {
        return non_constraint_based.add_manager<T, Manager>(name);
    }
    template <Joint T, kit::DerivedFrom<constraint_manager2D<T>> Manager> Manager *add_manager(const std::string &name)
    {
        return constraint_based.add_manager<T, Manager>(name);
    }

    template <Joint T> auto manager() const
    {
        if constexpr (std::is_base_of_v<constraint2D, T>)
            return constraint_based.manager<T>();
        else
            return non_constraint_based.manager<T>();
    }
    template <Joint T> auto manager()
    {
        if constexpr (std::is_base_of_v<constraint2D, T>)
            return constraint_based.manager<T>();
        else
            return non_constraint_based.manager<T>();
    }

    template <Joint T> bool remove()
    {
        if constexpr (std::is_base_of_v<constraint2D, T>)
            return constraint_based.remove<T>();
        else
            return non_constraint_based.remove<T>();
    }
    template <Joint T> bool remove(const std::size_t index)
    {
        if constexpr (std::is_base_of_v<constraint2D, T>)
            return constraint_based.remove<T>(index);
        else
            return non_constraint_based.remove<T>(index);
    }
    template <Joint T> bool remove(const T &joint)
    {
        if constexpr (std::is_base_of_v<constraint2D, T>)
            return constraint_based.remove(joint);
        else
            return non_constraint_based.remove(joint);
    }
    template <Joint T> bool remove(const typename T::id_type &id)
    {
        if constexpr (std::is_base_of_v<constraint2D, T>)
            return constraint_based.remove(id);
        else
            return non_constraint_based.remove(id);
    }

  private:
    joint_repository2D(world2D &world);

    friend class world2D;
};
} // namespace ppx