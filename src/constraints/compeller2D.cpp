#include "ppx/internal/pch.hpp"
#include "ppx/constraints/compeller2D.hpp"
#include "ppx/world2D.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
compeller2D::compeller2D(const world2D &parent, const std::size_t allocations) : m_parent(parent)
{
    m_constraints.reserve(allocations);
}

bool compeller2D::remove_constraint(std::size_t index, const kit::event<const constraint2D &> &event_callback)
{
    if (index >= m_constraints.size())
    {
        KIT_WARN("Constraint index exceeds array bounds. Aborting... - index: {0}, size: {1}", index,
                 m_constraints.size())
        return false;
    }
    event_callback(*m_constraints[index]);
    m_constraints.erase(m_constraints.begin() + (long)index);
    return true;
}
bool compeller2D::remove_constraint(const constraint2D *ctr, const kit::event<const constraint2D &> &event_callback)
{
    for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it)
        if (it->get() == ctr)
        {
            event_callback(*ctr);
            m_constraints.erase(it);
            return true;
        }
    return false;
}
bool compeller2D::remove_constraint(kit::uuid id, const kit::event<const constraint2D &> &event_callback)
{
    for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it)
        if ((*it)->id() == id)
        {
            event_callback(**it);
            m_constraints.erase(it);
            return true;
        }
    return false;
}

void compeller2D::clear_constraints(const kit::event<const constraint2D &> &event_callback)
{
    for (const auto &ctr : m_constraints)
        event_callback(*ctr);
    m_constraints.clear();
}

void compeller2D::validate(const kit::event<const constraint2D &> &event_callback)
{
    for (auto it = m_constraints.begin(); it != m_constraints.end();)
        if (!(*it)->valid())
        {
            event_callback(**it);
            it = m_constraints.erase(it);
        }
        else
            ++it;
}

void compeller2D::solve_and_load_constraints(std::vector<float> &stchanges,
                                             const kit::stack_vector<float> &inv_masses) const
{
    KIT_PERF_FUNCTION()
    if (m_constraints.empty())
        return;
    const kit::stack_vector<float> jcb = jacobian(), djcb = jacobian_derivative();
    const kit::stack_vector<float> A = lhs(jcb, inv_masses);
    const kit::stack_vector<float> b = rhs(jcb, djcb, stchanges, inv_masses);
    const kit::stack_vector<float> lambda = lu_decomposition(A, b);
    load_constraint_accels(jcb, lambda, stchanges);
}

const std::vector<kit::scope<constraint2D>> &compeller2D::constraints() const
{
    return m_constraints;
}

kit::stack_vector<float> compeller2D::constraint_matrix(const constraint_gradient_fun &constraint_gradients) const
{
    KIT_PERF_FUNCTION()
    const std::size_t rows = m_constraints.size(), cols = 3 * m_parent.size();
    kit::stack_vector<float> cmatrix(rows * cols, 0.f);

    for (std::size_t i = 0; i < rows; i++)
        for (const auto &[body, gradient] : constraint_gradients(*m_constraints[i]))
            for (std::size_t k = 0; k < 3; k++)
            {
                const std::size_t j = body->index() * 3 + k;
                cmatrix[i * cols + j] = gradient[k];
            }

    return cmatrix;
}

kit::stack_vector<float> compeller2D::jacobian() const
{
    return constraint_matrix(&constraint2D::constraint_gradients);
}
kit::stack_vector<float> compeller2D::jacobian_derivative() const
{
    return constraint_matrix(&constraint2D::constraint_derivative_gradients);
}

kit::stack_vector<float> compeller2D::lhs(const kit::stack_vector<float> &jcb,
                                          const kit::stack_vector<float> &inv_masses) const
{
    KIT_PERF_FUNCTION()
    const std::size_t rows = m_constraints.size(), cols = 3 * m_parent.size();
    kit::stack_vector<float> A(rows * rows, 0.f);
    for (std::size_t i = 0; i < rows; i++)
        for (std::size_t j = 0; j < rows; j++)
        {
            const std::size_t idx = i * rows + j;
            for (std::size_t k = 0; k < cols; k++)
            {
                if (kit::approaches_zero(inv_masses[k]))
                    continue;
                const std::size_t idx1 = i * cols + k, idx2 = j * cols + k;
                A[idx] += jcb[idx1] * jcb[idx2] * inv_masses[k];
            }
        }
    return A;
}

kit::stack_vector<float> compeller2D::rhs(const kit::stack_vector<float> &jcb, const kit::stack_vector<float> &djcb,
                                          const std::vector<float> &stchanges,
                                          const kit::stack_vector<float> &inv_masses) const
{
    KIT_PERF_FUNCTION()
    const std::size_t rows = m_constraints.size(), cols = 3 * m_parent.size();
    kit::stack_vector<float> b(rows, 0.f);

    for (std::size_t i = 0; i < rows; i++)
    {
        for (std::size_t j = 0; j < m_parent.size(); j++)
            for (std::size_t k = 0; k < 3; k++)
            {
                const std::size_t index1 = j * 3 + k, index2 = j * 6 + k;
                const std::size_t idx = i * cols + index1;

                const float to_substract =
                    djcb[idx] * stchanges[index2] + jcb[idx] * stchanges[index2 + 3] * inv_masses[index1];
                b[i] -= to_substract;
            }
        const float anti_drift = (m_constraints[i]->stiffness() * m_constraints[i]->constraint_value() +
                                  m_constraints[i]->dampening() * m_constraints[i]->constraint_derivative());
        b[i] -= anti_drift;
    }
    return b;
}

kit::stack_vector<float> compeller2D::lu_decomposition(const kit::stack_vector<float> &A,
                                                       const kit::stack_vector<float> &b) const
{
    KIT_PERF_FUNCTION()
    const std::size_t size = m_constraints.size();
    kit::stack_vector<float> sol(size, 0.f), L(size * size, 0.f), U(size * size, 0.f);
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
    for (std::size_t i = size - 1; i < size; i--)
    {
        float val = sol[i];
        for (std::size_t j = i + 1; j < size; j++)
            val -= U[i * size + j] * sol[j];
        sol[i] = val / U[i * size + i];
    }
    return sol;
}

void compeller2D::load_constraint_accels(const kit::stack_vector<float> &jcb, const kit::stack_vector<float> &lambda,
                                         std::vector<float> &stchanges) const
{
    KIT_PERF_FUNCTION()
    const std::size_t rows = m_constraints.size(), cols = 3 * m_parent.size();
    for (std::size_t i = 0; i < m_parent.size(); i++)
        for (std::size_t j = 0; j < 3; j++)
            for (std::size_t k = 0; k < rows; k++)
            {
                const std::size_t idx1 = 6 * i + j + 3, idx2 = k * cols + 3 * i + j;
                stchanges[idx1] += jcb[idx2] * lambda[k];
            }
}
} // namespace ppx