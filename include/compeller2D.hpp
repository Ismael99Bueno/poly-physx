#ifndef COMPELLER2D_HPP
#define COMPELLER2D_HPP

#include "entity2D.hpp"
#include "pass_key.hpp"
#include "engine_callbacks.hpp"
#include <vector>
#include <functional>

namespace ppx
{
    class constraint_interface2D;
    class compeller2D final
    {
    public:
        compeller2D(engine_key,
                    std::vector<entity2D> *entities,
                    std::size_t allocations,
                    engine_callbacks *cbs);

        void add_constraint(const std::shared_ptr<constraint_interface2D> &c);
        bool remove_constraint(const std::shared_ptr<const constraint_interface2D> &c);
        void clear_constraints();

        void validate();

        void solve_and_load_constraints(std::vector<float> &stchanges,
                                        const std::vector<float> &inv_masses) const;

        const std::vector<std::shared_ptr<constraint_interface2D>> &constraints() const;

    private:
        std::vector<entity2D> *m_entities;
        std::vector<std::shared_ptr<constraint_interface2D>> m_constraints;
        engine_callbacks *m_callbacks;

        using constraint_grad_fun = std::function<std::array<float, 3>(const constraint_interface2D &, entity2D &)>;
        std::vector<float> constraint_matrix(const constraint_grad_fun &constraint_grad) const;
        std::vector<float> jacobian() const;
        std::vector<float> jacobian_derivative() const;

        std::vector<float> lhs(const std::vector<float> &jcb,
                               const std::vector<float> &inv_masses) const;

        std::vector<float> rhs(const std::vector<float> &jcb,
                               const std::vector<float> &djcb,
                               const std::vector<float> &stchanges,
                               const std::vector<float> &inv_masses) const;

        std::vector<float> lu_decomposition(const std::vector<float> &A, const std::vector<float> &b) const;
        void load_constraint_accels(const std::vector<float> &jcb,
                                    const std::vector<float> &lambda,
                                    std::vector<float> &stchanges) const;

        compeller2D(const compeller2D &) = delete;
        compeller2D &operator=(const compeller2D &) = delete;
    };
}

#endif