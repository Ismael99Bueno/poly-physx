#ifndef PPX_COMPELLER2D_HPP
#define PPX_COMPELLER2D_HPP

#include "ppx/body2D.hpp"
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
class engine2D;

class compeller2D final : kit::non_copyable
{
  public:
    compeller2D(const engine2D &parent, std::size_t allocations);

    template <typename T, class... ConstraintArgs>
    T *add_constraint(const kit::event<constraint2D *> &event_callback, ConstraintArgs &&...args)
    {
        static_assert(std::is_base_of<constraint2D, T>::value, "Constraint must inherit from constraint2D!");
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

    void solve_and_load_constraints(std::vector<float> &stchanges, const kit::stack_vector<float> &inv_masses) const;

    const std::vector<kit::scope<constraint2D>> &constraints() const;

  private:
    const engine2D &m_parent;
    std::vector<kit::scope<constraint2D>> m_constraints;

    using constraint_gradient_fun = std::function<std::vector<constraint2D::body_gradient>(const constraint2D &)>;
    kit::stack_vector<float> constraint_matrix(const constraint_gradient_fun &constraint_grad) const;
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