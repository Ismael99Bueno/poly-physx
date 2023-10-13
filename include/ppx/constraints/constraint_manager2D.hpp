#ifndef PPX_CONSTRAINT_MANAGER2D_HPP
#define PPX_CONSTRAINT_MANAGER2D_HPP

#include "ppx/body2D.hpp"
#include "ppx/events/world_events.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/container/stack_vector.hpp"
#include "kit/memory/scope.hpp"
#include <vector>
#include <functional>
#include <memory>
#include <type_traits>

namespace ppx
{
class world2D;

class constraint_manager2D final : kit::non_copyable
{
  public:
    constraint_manager2D(std::size_t allocations);

    template <typename T, class... ConstraintArgs>
    T *add_constraint(const kit::event<constraint2D *> &event_callback, ConstraintArgs &&...args)
    {
        static_assert(std::is_base_of_v<constraint2D, T>, "Constraint must inherit from constraint2D!");
        auto ctr = kit::make_scope<T>(std::forward<ConstraintArgs>(args)...);
        T *ptr = ctr.get();

        m_constraints.push_back(std::move(ctr));
        event_callback(ptr);
        return ptr;
    }

    bool remove_constraint(std::size_t index, const kit::event<const constraint2D &> &event_callback);
    bool remove_constraint(const constraint2D *ctr, const kit::event<const constraint2D &> &event_callback);
    bool remove_constraint(kit::uuid id, const kit::event<const constraint2D &> &event_callback);

    void clear_constraints(const kit::event<const constraint2D &> &event_callback);

    void validate(const kit::event<const constraint2D &> &event_callback);

    void solve_and_load_constraints(std::vector<float> &state_derivative,
                                    const kit::stack_vector<float> &inv_masses) const;

    const std::vector<kit::scope<constraint2D>> &constraints() const;

  private:
    std::vector<kit::scope<constraint2D>> m_constraints;
};
} // namespace ppx

#endif