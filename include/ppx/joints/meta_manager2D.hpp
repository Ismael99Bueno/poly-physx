#pragma once

#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include <typeinfo>

namespace ppx
{
class constraint_driven_resolution2D;
class joint_driven_resolution2D;
template <typename T>
concept IManager = std::is_same_v<T, ijoint_manager2D> || std::is_same_v<T, iconstraint_manager2D>;

template <IManager IM> class meta_manager2D : public idmanager2D<kit::scope<IM>>
{
    meta_manager2D(world2D &world, joint_events &jevents) : idmanager2D<kit::scope<IM>>(world), jevents(jevents)
    {
    }

    joint_events &jevents;

    template <Joint2D T,
              kit::DerivedFrom<typename IM::template manager_t<T>> Manager = typename IM::template manager_t<T>>
    Manager *add_manager(const std::string &name)
    {
        KIT_ASSERT_ERROR(!this->contains(Manager::name()), "There is already a manager of this type in the repository")
        auto manager = kit::make_scope<Manager>(this->world, jevents, name);
        Manager *ptr = manager.get();
        this->m_elements.push_back(std::move(manager));
        return ptr;
    }

    template <Joint2D T,
              kit::DerivedFrom<typename IM::template manager_t<T>> Manager = typename IM::template manager_t<T>>
    const Manager *manager() const
    {
        const IM *manager = (*this)[Manager::name()];
        return manager ? static_cast<const Manager *>(manager) : nullptr;
    }
    template <Joint2D T,
              kit::DerivedFrom<typename IM::template manager_t<T>> Manager = typename IM::template manager_t<T>>
    Manager *manager()
    {
        IM *manager = (*this)[Manager::name()];
        return manager ? static_cast<Manager *>(manager) : nullptr;
    }

    using idmanager2D<kit::scope<IM>>::remove;
    bool remove(std::size_t index) override;
    bool remove(joint2D *joint);

    template <Joint2D T> bool remove()
    {
        using Manager = typename IM::template manager_t<T>;
        for (std::size_t i = 0; i < this->m_elements.size(); i++)
            if (this->m_elements[i]->id == Manager::name())
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

class joint_meta_manager2D final : public meta_manager2D<ijoint_manager2D>
{
    using meta_manager2D<ijoint_manager2D>::meta_manager2D;
    joint_driven_resolution2D *m_resolution = nullptr;

    void solve();
    friend class world2D;
    friend class collision_manager2D;
};

class constraint_meta_manager2D final : public meta_manager2D<iconstraint_manager2D>
{
    using meta_manager2D<iconstraint_manager2D>::meta_manager2D;
    constraint_driven_resolution2D *m_resolution = nullptr;

    void solve();
    friend class world2D;
    friend class collision_manager2D;
};
} // namespace ppx