#include "compeller2D.hpp"
#include "constrain2D.hpp"

#define ROWS m_constrains.size()
#define COLS (3 * m_entities.size())

namespace physics
{
    compeller2D::compeller2D(const std::vector<entity2D> &entities, const std::size_t allocations) : m_entities(entities)
    {
        m_constrains.reserve(allocations);
    }

    void compeller2D::add(const constrain2D *c) { m_constrains.push_back(c); }

    std::vector<float> compeller2D::solve_constrains(const std::vector<float> &state, const std::vector<float> &stchanges) const
    {
        const std::vector<float> jcb = jacobian(), djcb = jacobian_derivative();
        const std::vector<float> A = lhs(jcb);
        const std::vector<float> b = rhs(jcb, djcb, state, stchanges);
        const std::vector<float> lambda = lu_decomposition(A, b);
        return constrain_accels(jcb, lambda);
    }

    std::vector<float> compeller2D::constrain_matrix(std::array<float, 3> (constrain2D::*constrain)(const std::vector<const_entity_ptr> &) const) const
    {
        std::vector<float> cmatrix(ROWS * COLS, 0.f);
        for (std::size_t i = 0; i < ROWS; i++)
            for (std::size_t j = 0; j < COLS; j++)
                if (m_constrains[i]->contains({m_entities, j}))
                {
                    const std::array<float, 3> state = (m_constrains[i]->*constrain)(m_constrains[i]->m_entities);
                    for (std::size_t k = 0; k < 3; k++)
                        cmatrix[i * ROWS + j * 3 + k] = state[k];
                }
        return cmatrix;
    }

    std::vector<float> compeller2D::jacobian() const { return constrain_matrix(&constrain2D::constrain_grad); }
    std::vector<float> compeller2D::jacobian_derivative() const { return constrain_matrix(&constrain2D::constrain_grad_dt); }

    std::vector<float> compeller2D::lhs(const std::vector<float> &jcb) const
    {
        std::vector<float> A(ROWS * ROWS, 0.f);
        for (std::size_t i = 0; i < ROWS; i++)
            for (std::size_t j = 0; j < ROWS; j++)
            {
                const std::size_t id = i * ROWS + j;
                for (std::size_t k = 0; k < COLS; k++)
                {
                    const std::size_t id1 = i * ROWS + k, id2 = k * COLS + j;
                    A[id] += jcb[id1] * jcb[id2];
                }
            }
        return A;
    }

    std::vector<float> compeller2D::rhs(const std::vector<float> &jcb,
                                        const std::vector<float> &djcb,
                                        const std::vector<float> &state,
                                        const std::vector<float> &stchanges) const
    {
        std::vector<float> b(ROWS, 0.f), qdot(ROWS, 0.f), accels(ROWS, 0.f);
        std::size_t index = 0;
        for (std::size_t i = 0; i < m_entities.size(); i++)
            for (std::size_t j = 3; j < 6; j++)
            {
                qdot[index] = state[6 * i + j];
                accels[index++] = stchanges[6 * i + j];
            }

        for (std::size_t i = 0; i < ROWS; i++)
            for (std::size_t j = 0; j < COLS; j++)
            {
                const std::size_t id = i * ROWS + j;
                b[i] -= djcb[id] * qdot[i] + jcb[id] * accels[i];
            }
        return b;
    }

    std::vector<float> compeller2D::lu_decomposition(const std::vector<float> &A,
                                                     const std::vector<float> &b) const
    {
        const std::size_t size = ROWS;
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

    std::vector<float> compeller2D::constrain_accels(const std::vector<float> &jcb,
                                                     const std::vector<float> &lambda) const
    {
        std::vector<float> accels(ROWS, 0.f);
        for (std::size_t i = 0; i < ROWS; i++)
            for (std::size_t j = 0; j < ROWS; j++)
                accels[i] += lambda[i * ROWS + j];
        return accels;
    }
}