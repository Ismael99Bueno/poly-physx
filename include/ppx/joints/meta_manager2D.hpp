#pragma once

#include "ppx/joints/joint_manager2D.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include <typeinfo>

namespace ppx
{
class sequential_impulses_resolution2D;
template <typename T>
concept IManager = std::is_same_v<T, ijoint_manager2D> || std::is_same_v<T, iconstraint_manager2D>;

template <IManager IM> class meta_manager2D : public idmanager2D<kit::scope<IM>>
{
  public:
    using idmanager2D<kit::scope<IM>>::idmanager2D;

    template <Joint T> T *add(const typename T::specs &spc)
    {
        using Manager = typename IM::template manager_t<T>;
        Manager *mng = manager<T>();
        if (!mng)
            mng = add_manager<T>(typeid(T).name());
        return mng->add(spc);
    }

    template <Joint T,
              kit::DerivedFrom<typename IM::template manager_t<T>> Manager = typename IM::template manager_t<T>>
    Manager *add_manager(const std::string &name)
    {
        KIT_ASSERT_ERROR(!this->contains(Manager::name()), "There is already a solver of this type in the repository")
        auto manager = kit::make_scope<Manager>(this->world, name);
        Manager *ptr = manager.get();
        this->m_elements.push_back(std::move(manager));
        return ptr;
    }

    template <Joint T,
              kit::DerivedFrom<typename IM::template manager_t<T>> Manager = typename IM::template manager_t<T>>
    const Manager *manager() const
    {
        const IM *solver = (*this)[Manager::name()];
        return solver ? static_cast<const Manager *>(solver) : nullptr;
    }
    template <Joint T,
              kit::DerivedFrom<typename IM::template manager_t<T>> Manager = typename IM::template manager_t<T>>
    Manager *manager()
    {
        IM *solver = (*this)[Manager::name()];
        return solver ? static_cast<Manager *>(solver) : nullptr;
    }

    using idmanager2D<kit::scope<IM>>::remove;
    bool remove(std::size_t index) override;

    template <Joint T> bool remove()
    {
        using Manager = typename IM::template manager_t<T>;
        for (std::size_t i = 0; i < this->m_elements.size(); i++)
            if (this->m_elements[i]->id == Manager::name())
                return remove(i);
        return false;
    }
    template <Joint T> bool remove(const std::size_t index)
    {
        using Manager = typename IM::template manager_t<T>;
        Manager *mng = manager<T>();
        return mng ? mng->remove(index) : false;
    }
    template <Joint T> bool remove(const T *element)
    {
        using Manager = typename IM::template manager_t<T>;
        Manager *mng = manager<T>();
        return mng ? mng->remove(element) : false;
    }

  private:
    void on_body_removal_validation(const body2D *body)
    {
        for (const auto &solver : this->m_elements)
            solver->on_body_removal_validation(body);
    }

    friend class world2D;
};

class joint_meta_manager2D final : public meta_manager2D<ijoint_manager2D>
{
    using meta_manager2D<ijoint_manager2D>::meta_manager2D;
    void solve();
    friend class world2D;
    friend class joint_repository2D;
};

class constraint_meta_manager2D final : public meta_manager2D<iconstraint_manager2D>
{
    using meta_manager2D<iconstraint_manager2D>::meta_manager2D;
    sequential_impulses_resolution2D *m_si_solver = nullptr;

    void solve();
    void delegate_contacts_resolution(sequential_impulses_resolution2D *solver);

    friend class world2D;
    friend class joint_repository2D;
    friend class sequential_impulses_resolution2D;
};
} // namespace ppx