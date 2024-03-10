#pragma once

#include "ppx/manager2D.hpp"
#include <typeinfo>

namespace ppx
{
class joint_solver2D;
class constraint_solver2D;

template <typename T>
concept Solver = std::is_same_v<T, joint_solver2D> || std::is_same_v<T, constraint_solver2D>;

template <Solver S> class meta_manager2D : public manager2D<kit::scope<S>>
{
  public:
    using manager2D<kit::scope<S>>::manager2D;

    template <typename T> T &add(const typename T::specs &spc)
    {
        using Manager = typename S::template manager_t<T>;
        Manager *mng = manager<T>();
        if (!mng)
            mng = add_manager<T>(typeid(T).name());
        return mng->add(spc);
    }

    template <typename T,
              kit::DerivedFrom<typename S::template manager_t<T>> Manager = typename S::template manager_t<T>>
    Manager *add_manager(const std::string &name)
    {
        KIT_ASSERT_ERROR(!this->contains(Manager::name()), "There is already a solver of this type in the repository")
        auto manager = kit::make_scope<Manager>(this->world, name);
        Manager *ptr = manager.get();
        this->m_elements.push_back(std::move(manager));
        return ptr;
    }

    template <typename T,
              kit::DerivedFrom<typename S::template manager_t<T>> Manager = typename S::template manager_t<T>>
    const Manager *manager() const
    {
        const S *solver = (*this)[Manager::name()];
        return solver ? static_cast<const Manager *>(solver) : nullptr;
    }
    template <typename T,
              kit::DerivedFrom<typename S::template manager_t<T>> Manager = typename S::template manager_t<T>>
    Manager *manager()
    {
        S *solver = (*this)[Manager::name()];
        return solver ? static_cast<Manager *>(solver) : nullptr;
    }

    using manager2D<kit::scope<S>>::remove;
    bool remove(std::size_t index) override;

    template <typename T> bool remove()
    {
        using Manager = typename S::template manager_t<T>;
        for (std::size_t i = 0; i < this->m_elements.size(); i++)
            if (this->m_elements[i]->id == Manager::name())
                return remove(i);
        return false;
    }
    template <typename T> bool remove(const std::size_t index)
    {
        using Manager = typename S::template manager_t<T>;
        Manager *mng = manager<T>();
        return mng ? mng->remove(index) : false;
    }
    template <typename T> bool remove(const T &element)
    {
        return remove<T>(element.index);
    }
    template <typename T> bool remove(const typename T::id_type &id)
    {
        using Manager = typename S::template manager_t<T>;
        Manager *mng = manager<T>();
        return mng ? mng->remove(id) : false;
    }
};
} // namespace ppx