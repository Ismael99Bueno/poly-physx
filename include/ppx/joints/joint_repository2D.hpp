#pragma once

#include "ppx/joints/joint_meta_manager2D.hpp"
#include "ppx/joints/island2D.hpp"

namespace ppx
{
class joint_repository2D final : public manager2D<joint2D>
{
  public:
    actuator_meta_manager2D actuators;
    constraint_meta_manager2D constraints;

    void solve_islands();

    bool islands_enabled() const;
    void islands_enabled(bool enable);

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
    using manager2D<joint2D>::remove;
    bool remove(std::size_t index) override;

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

  private:
    joint_repository2D(world2D &world);

    island2D *create_island();
    void try_split_islands(std::uint32_t max_tries);
    bool split_island(island2D *island);
    void build_islands_from_existing_simulation();

    std::vector<island2D *> m_islands;
    bool m_enable_islands = true;
    std::size_t m_island_to_split = 0;

    friend class world2D;
    friend class body_manager2D;
    friend class body2D;
};
} // namespace ppx