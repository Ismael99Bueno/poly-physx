#pragma once

#include "ppx/joints/meta_manager2D.hpp"

namespace ppx
{
class joint_repository2D
{
  public:
    joint_meta_manager2D non_constraint_based;
    constraint_meta_manager2D constraint_based;

    struct
    {
        kit::event<joint2D *> on_addition;
        kit::event<joint2D &> on_removal;
    } events;

    template <Joint2D T> T *add(const typename T::specs &spc)
    {
        auto mng = manager<T>();
        KIT_ASSERT_ERROR(mng, "There is no manager of this type in the repository")
        T *joint = mng->add(spc);
        events.on_addition(joint);
        return joint;
    }

    template <Joint2D T> auto add_manager(const std::string &name)
    {
        if constexpr (VConstraint2D<T>)
            return add_manager<T, constraint_manager2D<T>>(name);
        else
            return add_manager<T, joint_manager2D<T>>(name);
    }

    template <Joint2D T, kit::DerivedFrom<joint_manager2D<T>> Manager> Manager *add_manager(const std::string &name)
    {
        return non_constraint_based.add_manager<T, Manager>(name);
    }
    template <Joint2D T, kit::DerivedFrom<constraint_manager2D<T>> Manager>
    Manager *add_manager(const std::string &name)
    {
        return constraint_based.add_manager<T, Manager>(name);
    }

    template <Joint2D T> auto manager() const
    {
        if constexpr (VConstraint2D<T>)
            return constraint_based.manager<T>();
        else
            return non_constraint_based.manager<T>();
    }
    template <Joint2D T> auto manager()
    {
        if constexpr (VConstraint2D<T>)
            return constraint_based.manager<T>();
        else
            return non_constraint_based.manager<T>();
    }

    template <Joint2D T> bool remove_manager()
    {
        if constexpr (VConstraint2D<T>)
            return constraint_based.remove_manager<T>();
        else
            return non_constraint_based.remove_manager<T>();
    }
    bool remove(const joint2D *joint);
    template <Joint2D T> bool remove(const std::size_t index)
    {
        auto mng = manager<T>();
        if (!mng || index >= mng->size())
            return false;
        T *joint = mng->at(index);
        mng->events.on_removal(*joint);
        return mng->remove(index);
    }
    template <Joint2D T> bool remove(const T *joint)
    {
        auto mng = manager<T>();
        if (!mng || !mng->contains(joint))
            return false;
        mng->events.on_removal(*joint);
        return mng->remove(joint);
    }

  private:
    joint_repository2D(world2D &world);

    friend class world2D;
};
} // namespace ppx