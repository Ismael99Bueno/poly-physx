#ifndef PPX_COMPELLER2D_HPP
#define PPX_COMPELLER2D_HPP

#include "ppx/entity2D.hpp"
#include "ppx/events/engine_events.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/container/stack_vector.hpp"
#include "kit/memory/scope.hpp"
#include <vector>
#include <functional>
#include <memory>
#include <type_traits>

namespace ppx
{
class constraint_interface2D;
class engine2D;

class compeller2D final : kit::non_copyable
{
  public:
    compeller2D(const engine2D &parent, std::size_t allocations, engine_events *cbs);

    template <typename T, class... ConstraintArgs> T *add_constraint(ConstraintArgs &&...args)
    {
        static_assert(std::is_base_of<constraint_interface2D, T>::value, "Constraint must inherit from constraint2D!");
        auto ctr = kit::make_scope<T>(std::forward<ConstraintArgs>(args)...);
        T *ptr = ctr.get();

        m_constraints.push_back(std::move(ctr));
        m_callbacks->on_constraint_addition(ptr);
        return ptr;
    }
    bool remove_constraint(const constraint_interface2D *ctr);
    void clear_constraints();

    void validate();

    void solve_and_load_constraints(std::vector<float> &stchanges, const kit::stack_vector<float> &inv_masses) const;

    const std::vector<kit::scope<constraint_interface2D>> &constraints() const;

  private:
    const engine2D &m_parent;
    std::vector<kit::scope<constraint_interface2D>> m_constraints;
    engine_events *m_callbacks;

    using constraint_grad_fun = std::function<std::array<float, 3>(const constraint_interface2D &, entity2D &)>;
    kit::stack_vector<float> constraint_matrix(const constraint_grad_fun &constraint_grad) const;
    kit::stack_vector<float> jacobian() const;
    kit::stack_vector<float> jacobian_derivative() const;

    kit::stack_vector<float> lhs(const kit::stack_vector<float> &jcb, const kit::stack_vector<float> &inv_masses) const;

    kit::stack_vector<float> rhs(const kit::stack_vector<float> &jcb, const kit::stack_vector<float> &djcb,
                                 const std::vector<float> &stchanges, const kit::stack_vector<float> &inv_masses) const;

    kit::stack_vector<float> lu_decomposition(const kit::stack_vector<float> &A,
                                              const kit::stack_vector<float> &b) const;
    void load_constraint_accels(const kit::stack_vector<float> &jcb, const kit::stack_vector<float> &lambda,
                                std::vector<float> &stchanges) const;
};
} // namespace ppx

#endif