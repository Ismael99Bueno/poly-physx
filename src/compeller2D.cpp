#include "compeller2D.hpp"
#include "perf.hpp"
#include "debug.hpp"
#include "constraint_interface2D.hpp"

namespace phys
{
    compeller2D::compeller2D(std::vector<entity2D> &entities,
                             const std::size_t allocations) : m_entities(entities)
    {
        m_constraints.reserve(allocations);
    }

    void compeller2D::add_constraint(const std::shared_ptr<constraint_interface2D> &c) { m_constraints.emplace_back(c); }
    void compeller2D::remove_constraint(const std::shared_ptr<const constraint_interface2D> &c)
    {
        m_constraints.erase(std::remove(m_constraints.begin(), m_constraints.end(), c), m_constraints.end());
    }
    void compeller2D::clear_constraints() { m_constraints.clear(); }

    void compeller2D::validate()
    {
        for (auto it = m_constraints.begin(); it != m_constraints.end();)
            if (!((*it)->try_validate()))
                it = m_constraints.erase(it);
            else
                ++it;
    }

    void compeller2D::solve_and_load_constraints(std::vector<float> &stchanges, const std::vector<float> &inv_masses) const
    {
        PERF_FUNCTION()
        if (m_constraints.empty())
            return;
        const std::vector<float> jcb = jacobian(), djcb = jacobian_derivative();
        const std::vector<float> A = lhs(jcb, inv_masses);
        const std::vector<float> b = rhs(jcb, djcb, stchanges, inv_masses);
        const std::vector<float> lambda = lu_decomposition(A, b);
        return load_constraint_accels(jcb, lambda, stchanges);
    }

    const std::vector<std::shared_ptr<constraint_interface2D>> &compeller2D::constraints() const { return m_constraints; }

    std::vector<float> compeller2D::constraint_matrix(const constraint_grad_fun &constraint_grad) const
    {
        PERF_FUNCTION()
        const std::size_t rows = m_constraints.size(), cols = 3 * m_entities.size();
        std::vector<float> cmatrix(rows * cols, 0.f);
        for (std::size_t i = 0; i < rows; i++)
            for (std::size_t ct_idx = 0; ct_idx < m_constraints[i]->size(); ct_idx++)
            {
                entity2D &e = (*m_constraints[i])[ct_idx];
                const std::array<float, 3> state = constraint_grad(*m_constraints[i], e);

                for (std::size_t k = 0; k < 3; k++)
                {
                    const std::size_t j = e.index() * 3 + k;
                    cmatrix[i * cols + j] = state[k];
                }
            }
        return cmatrix;
    }

    std::vector<float> compeller2D::jacobian() const { return constraint_matrix(&constraint_interface2D::constraint_grad); }
    std::vector<float> compeller2D::jacobian_derivative() const { return constraint_matrix(&constraint_interface2D::constraint_grad_derivative); }

    std::vector<float> compeller2D::lhs(const std::vector<float> &jcb,
                                        const std::vector<float> &inv_masses) const
    {
        PERF_FUNCTION()
        const std::size_t rows = m_constraints.size(), cols = 3 * m_entities.size();
        std::vector<float> A(rows * rows, 0.f);
        for (std::size_t i = 0; i < rows; i++)
            for (std::size_t j = 0; j < rows; j++)
            {
                const std::size_t id = i * rows + j;
                for (std::size_t k = 0; k < cols; k++)
                {
                    const std::size_t id1 = i * cols + k, id2 = j * cols + k;
                    A[id] += jcb[id1] * jcb[id2] * inv_masses[k];
                }
            }
        return A;
    }

    std::vector<float> compeller2D::rhs(const std::vector<float> &jcb,
                                        const std::vector<float> &djcb,
                                        const std::vector<float> &stchanges,
                                        const std::vector<float> &inv_masses) const
    {
        PERF_FUNCTION()
        const std::size_t rows = m_constraints.size(), cols = 3 * m_entities.size();
        std::vector<float> b(rows, 0.f);

        for (std::size_t i = 0; i < rows; i++)
        {
            for (std::size_t j = 0; j < m_entities.size(); j++)
                for (std::size_t k = 0; k < 3; k++)
                {
                    const std::size_t index1 = j * 3 + k, index2 = j * 6 + k;
                    const std::size_t id = i * cols + index1;

                    const float to_substract = djcb[id] * stchanges[index2] +
                                               jcb[id] * stchanges[index2 + 3] * inv_masses[index1];
                    b[i] -= to_substract;
                }
            const float anti_drift = (m_constraints[i]->stiffness() * m_constraints[i]->value() +
                                      m_constraints[i]->dampening() * m_constraints[i]->derivative());
            b[i] -= anti_drift;
        }
        return b;
    }

    std::vector<float> compeller2D::lu_decomposition(const std::vector<float> &A,
                                                     const std::vector<float> &b) const
    {
        PERF_FUNCTION()
        const std::size_t size = m_constraints.size();
        std::vector<float> L(size * size, 0.f), U(size * size, 0.f), sol(size, 0.f);
        for (std::size_t i = 0; i < size; i++)
        {
            for (std::size_t j = i; j < size; j++)
            {
                float sum = 0.f;
                for (std::size_t k = 0; k < i; k++)
                    sum += L[i * size + k] * U[k * size + j];
                U[i * size + j] = A[i * size + j] - sum;
            }

            L[i * size + i] = 1.f;
            for (std::size_t j = i + 1; j < size; j++)
            {
                float sum = 0.f;
                for (std::size_t k = 0; k < i; k++)
                    sum += L[j * size + k] * U[k * size + i];
                L[j * size + i] = (A[j * size + i] - sum) / U[i * size + i];
            }
        }

        for (std::size_t i = 0; i < size; i++)
        {
            float val = b[i];
            for (std::size_t j = 0; j < i; j++)
                val -= L[i * size + j] * sol[j];
            sol[i] = val / L[i * size + i];
        }
        for (std::size_t i = size - 1; i >= 0 && i < size; i--)
        {
            float val = sol[i];
            for (std::size_t j = i + 1; j < size; j++)
                val -= U[i * size + j] * sol[j];
            sol[i] = val / U[i * size + i];
        }
        return sol;
    }

    void compeller2D::load_constraint_accels(const std::vector<float> &jcb,
                                             const std::vector<float> &lambda,
                                             std::vector<float> &stchanges) const
    {
        PERF_FUNCTION()
        const std::size_t rows = m_constraints.size(), cols = 3 * m_entities.size();
        for (std::size_t i = 0; i < m_entities.size(); i++)
            if (m_entities[i].dynamic())
                for (std::size_t j = 0; j < 3; j++)
                    for (std::size_t k = 0; k < rows; k++)
                    {
                        const std::size_t id1 = 6 * i + j + 3,
                                          id2 = k * cols + 3 * i + j;
                        stchanges[id1] += jcb[id2] * lambda[k];
                    }
    }
}