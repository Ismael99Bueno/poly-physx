#ifndef COMPELLER2D_HPP
#define COMPELLER2D_HPP
#include "ppx/core.hpp"

#include "ppx/entity2D.hpp"
#include "ppx/engine_events.hpp"
#include "mem/stack_allocator.hpp"
#include <vector>
#include <functional>
#include <memory>
#include <type_traits>

namespace ppx
{
class constraint_interface2D;
class compeller2D final
{
  public:
    compeller2D(std::vector<entity2D> *entities, std::size_t allocations, engine_events *cbs);

    template <typename T, class... Args> ref<T> add_constraint(Args &&...args)
    {
        static_assert(std::is_base_of<constraint_interface2D, T>::value, "Constraint must inherit from constraint2D!");
        const auto ctr = make_ref<T>(std::forward<Args>(args)...);
        m_constraints.push_back(ctr);
        m_callbacks->on_constraint_addition(ctr);
        return ctr;
    }
    bool remove_constraint(const ref<const constraint_interface2D> &ctr);
    void clear_constraints();

    void validate();

    void solve_and_load_constraints(std::vector<float> &stchanges, const stk_vector<float> &inv_masses) const;

    const std::vector<ref<constraint_interface2D>> &constraints() const;

  private:
    std::vector<entity2D> *m_entities;
    std::vector<ref<constraint_interface2D>> m_constraints;
    engine_events *m_callbacks;

    using constraint_grad_fun = std::function<std::array<float, 3>(const constraint_interface2D &, entity2D &)>;
    stk_vector<float> constraint_matrix(const constraint_grad_fun &constraint_grad) const;
    stk_vector<float> jacobian() const;
    stk_vector<float> jacobian_derivative() const;

    stk_vector<float> lhs(const stk_vector<float> &jcb, const stk_vector<float> &inv_masses) const;

    stk_vector<float> rhs(const stk_vector<float> &jcb, const stk_vector<float> &djcb,
                          const std::vector<float> &stchanges, const stk_vector<float> &inv_masses) const;

    stk_vector<float> lu_decomposition(const stk_vector<float> &A, const stk_vector<float> &b) const;
    void load_constraint_accels(const stk_vector<float> &jcb, const stk_vector<float> &lambda,
                                std::vector<float> &stchanges) const;

    compeller2D(const compeller2D &) = delete;
    compeller2D &operator=(const compeller2D &) = delete;
};
} // namespace ppx

#endif