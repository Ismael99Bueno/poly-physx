#pragma once

#include "ppx/joints/meta_manager2D.hpp"
#include "ppx/joints/island2D.hpp"

namespace ppx
{
class joint_repository2D final : public manager2D<joint2D>
{
  public:
    joint_meta_manager2D non_constraint_based;
    constraint_meta_manager2D constraint_based;

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
            return constraint_based.remove<T>();
        else
            return non_constraint_based.remove<T>();
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