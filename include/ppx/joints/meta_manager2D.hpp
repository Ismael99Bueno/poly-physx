#pragma once

#include "ppx/manager2D.hpp"
#include "ppx/body/body2D.hpp"
#include <typeinfo>

namespace ppx
{
class joint_solver2D;
class constraint_solver2D;

template <typename T>
concept Solver = std::is_same_v<T, joint_solver2D> || std::is_same_v<T, constraint_solver2D>;

template <Solver S> class meta_manager2D : public idmanager2D<kit::scope<S>>
{
  public:
    using idmanager2D<kit::scope<S>>::idmanager2D;

    template <typename T> T *add(const typename T::specs &spc)
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

    using idmanager2D<kit::scope<S>>::remove;
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

    void solve()
    {
        if constexpr (std::is_same_v<S, joint_solver2D>)
        {
            KIT_PERF_SCOPE("Joints solve")
            for (const auto &solver : this->m_elements)
                solver->solve();
        }
        else
        {
            KIT_ASSERT_ERROR(this->world.constraints.baumgarte_coef >= 0.f, "Baumgarte coef must be non-negative: {0}",
                             this->world.constraints.baumgarte_coef)
            KIT_ASSERT_ERROR(this->world.constraints.baumgarte_threshold >= 0.f,
                             "Baumgarte threshold must be non-negative: {0}",
                             this->world.constraints.baumgarte_threshold)
            KIT_ASSERT_ERROR(this->world.constraints.iterations > 0, "Iterations must be positive: {0}",
                             this->world.constraints.iterations)

            KIT_PERF_SCOPE("Constraints solve")
            for (std::size_t i = 0; i < this->world.constraints.velocity_iterations; i++)
                for (const auto &solver : this->m_elements)
                    solver->solve();
            for (std::size_t i = 0; i < this->world.constraints.position_iterations; i++)
            {
                bool fully_adjusted = true;
                for (const auto &solver : this->m_elements)
                    fully_adjusted &= solver->adjust_positions();
                if (fully_adjusted)
                    return;
            }
        }
    }

    void on_body_removal_validation(const body2D *body)
    {
        for (const auto &solver : this->m_elements)
            solver->on_body_removal_validation(body);
    }
};
} // namespace ppx