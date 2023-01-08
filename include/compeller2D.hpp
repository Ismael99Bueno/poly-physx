#ifndef COMPELLER2D_HPP
#define COMPELLER2D_HPP

#include "entity2D.hpp"
#include <vector>
#include <functional>

namespace phys
{
    class constraint_interface;
    class compeller2D
    {
    public:
        compeller2D() = delete;
        compeller2D(std::vector<entity2D> &entities,
                    std::size_t allocations);

        void add_constraint(const std::shared_ptr<constraint_interface> &c);
        void remove_constraint(const std::shared_ptr<const constraint_interface> &c);
        void clear_constraints();

        bool validate();

        void solve_and_load_constraints(std::vector<float> &stchanges,
                                        const std::vector<float> &inv_masses) const;

        const std::vector<std::shared_ptr<constraint_interface>> &constraints() const;

    private:
        std::vector<entity2D> &m_entities;
        std::vector<std::shared_ptr<constraint_interface>> m_constraints;

        using constraint_grad_fun = std::function<std::array<float, 3>(const constraint_interface &, entity2D &)>;
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
    };
}

#endif