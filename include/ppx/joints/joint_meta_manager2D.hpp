#pragma once

#include "ppx/actuators/actuator_manager2D.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include <typeinfo>

namespace ppx
{
class contact_constraint_solver2D;
class contact_actuator_solver2D;
template <typename T>
concept IManager = std::is_same_v<T, iactuator_manager2D> || std::is_same_v<T, iconstraint_manager2D>;

template <IManager IM> class joint_meta_manager2D : public idmanager2D<kit::scope<IM>>
{
    template <Joint2D T> using manager_t = typename IM::template manager_t<T>;

    joint_meta_manager2D(world2D &world, std::vector<joint2D *> &total_joints, manager_events<joint2D> &jevents)
        : idmanager2D<kit::scope<IM>>(world), m_total_joints(total_joints), m_jevents(jevents)
    {
    }

    std::vector<joint2D *> &m_total_joints;
    manager_events<joint2D> &m_jevents;

    template <Joint2D T, kit::DerivedFrom<manager_t<T>> Manager = manager_t<T>>
    Manager *add_manager(const std::string &name)
    {
        KIT_ASSERT_ERROR(!this->contains(Manager::name()), "There is already a manager of this type in the repository")
        auto manager = kit::make_scope<Manager>(this->world, m_total_joints, m_jevents, name);
        Manager *ptr = manager.get();
        this->m_elements.push_back(std::move(manager));
        return ptr;
    }

    template <Joint2D T, kit::DerivedFrom<manager_t<T>> Manager = manager_t<T>> const Manager *manager() const
    {
        const IM *manager = (*this)[Manager::name()];
        return manager ? static_cast<const Manager *>(manager) : nullptr;
    }
    template <Joint2D T, kit::DerivedFrom<manager_t<T>> Manager = manager_t<T>> Manager *manager()
    {
        IM *manager = (*this)[Manager::name()];
        return manager ? static_cast<Manager *>(manager) : nullptr;
    }

    using idmanager2D<kit::scope<IM>>::remove;
    bool remove(std::size_t index) override final;
    bool remove(joint2D *joint);

    template <Joint2D T> bool remove()
    {
        for (std::size_t i = 0; i < this->m_elements.size(); i++)
            if (this->m_elements[i]->id == manager_t<T>::name())
                return remove(i);
        return false;
    }

    void on_body_removal_validation(body2D *body)
    {
        for (const auto &manager : this->m_elements)
            manager->on_body_removal_validation(body);
    }

    friend class world2D;
    friend class joint_repository2D;
};

class actuator_meta_manager2D final : public joint_meta_manager2D<iactuator_manager2D>
{
    using joint_meta_manager2D<iactuator_manager2D>::joint_meta_manager2D;
    contact_actuator_solver2D *m_contact_solver = nullptr;

    void solve();
    friend class world2D;
    friend class collision_manager2D;
};

class constraint_meta_manager2D final : public joint_meta_manager2D<iconstraint_manager2D>
{
  public:
    specs::joint_manager2D::constraints2D params;

  private:
    using joint_meta_manager2D<iconstraint_manager2D>::joint_meta_manager2D;
    void solve();

    contact_constraint_solver2D *m_contact_solver = nullptr;

    friend class world2D;
    friend class collision_manager2D;
};
} // namespace ppx