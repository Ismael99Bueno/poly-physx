#include "compeller2D.hpp"
#include "constrain2D.hpp"

namespace physics
{
    compeller2D::compeller2D(const std::vector<entity2D> &entities,
                             const float stiffness,
                             const float dampening,
                             const std::size_t allocations) : m_entities(entities),
                                                              m_stiffness(stiffness),
                                                              m_dampening(dampening)
    {
        m_constrains.reserve(allocations);
    }

    void compeller2D::add(const constrain2D &c) { m_constrains.push_back(&c); }

    std::vector<float> compeller2D::solve_constrains(const std::vector<float> &stchanges) const
    {
        const std::vector<float> jcb = jacobian(), djcb = jacobian_derivative();
        const std::vector<float> A = lhs(jcb);
        const std::vector<float> b = rhs(jcb, djcb, stchanges);
        const std::vector<float> lambda = lu_decomposition(A, b);
        return constrain_accels(jcb, lambda);
    }

    std::vector<float> compeller2D::constrain_matrix(std::array<float, POS_PER_ENTITY> (constrain2D::*constrain_grad)(std::size_t) const) const
    {
        const std::size_t rows = m_constrains.size(), cols = POS_PER_ENTITY * m_entities.size();
        std::vector<float> cmatrix(rows * cols, 0.f);
        for (std::size_t i = 0; i < rows; i++)
        {
            std::size_t index = 0;
            for (std::size_t j = 0; j < m_entities.size(); j++)
                if (m_constrains[i]->contains({m_entities, j})) // ENTITIES MUST BE ADDED IN ORDER FOR THIS TO WORK
                {
                    const std::array<float, POS_PER_ENTITY> state = (m_constrains[i]->*constrain_grad)(index++); // PASS FUCKING ENTITY PTR AND DELETE GRAD_ENTITIES FROM CONSTRAIN
                    for (std::size_t k = 0; k < POS_PER_ENTITY; k++)
                        cmatrix[i * rows + j * POS_PER_ENTITY + k] = state[k];
                }
        }
        return cmatrix;
    }

    std::vector<float> compeller2D::jacobian() const { return constrain_matrix(&constrain2D::constrain_grad); }
    std::vector<float> compeller2D::jacobian_derivative() const { return constrain_matrix(&constrain2D::constrain_grad_derivative); }

    std::vector<float> compeller2D::lhs(const std::vector<float> &jcb) const
    {
        const std::size_t rows = m_constrains.size(), cols = POS_PER_ENTITY * m_entities.size();
        std::vector<float> A(rows * rows, 0.f);
        for (std::size_t i = 0; i < rows; i++)
            for (std::size_t j = 0; j < rows; j++)
            {
                const std::size_t id = i * rows + j;
                for (std::size_t k = 0; k < cols; k++)
                {
                    const std::size_t id1 = i * rows + k, id2 = j * rows + k;
                    A[id] += jcb[id1] * jcb[id2];
                }
            }
        return A;
    }

    std::vector<float> compeller2D::rhs(const std::vector<float> &jcb,
                                        const std::vector<float> &djcb,
                                        const std::vector<float> &stchanges) const
    {
        const std::size_t rows = m_constrains.size(), cols = POS_PER_ENTITY * m_entities.size();
        std::vector<float> b(rows, 0.f), qdot(stchanges.size() / 2, 0.f), accels(stchanges.size() / 2, 0.f);

        std::size_t index = 0;
        for (std::size_t i = 0; i < m_entities.size(); i++)
            for (std::size_t j = 0; j < POS_PER_ENTITY; j++)
            {
                qdot[index] = stchanges[6 * i + j];
                accels[index++] = stchanges[6 * i + j + POS_PER_ENTITY];
            }

        for (std::size_t i = 0; i < rows; i++)
        {
            for (std::size_t j = 0; j < cols; j++)
            {
                const std::size_t id = i * rows + j;
                b[i] -= (djcb[id] * qdot[j] + jcb[id] * accels[j]);
            }
            const float val = (m_stiffness * m_constrains[i]->value() + m_dampening * m_constrains[i]->derivative());
            b[i] -= val;
        }
        return b;
    }

    std::vector<float> compeller2D::lu_decomposition(const std::vector<float> &A,
                                                     const std::vector<float> &b) const
    {
        const std::size_t rows = m_constrains.size(), cols = POS_PER_ENTITY * m_entities.size();
        const std::size_t size = rows;
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
        const std::size_t rows = m_constrains.size(), cols = POS_PER_ENTITY * m_entities.size();
        std::vector<float> accels(cols, 0.f);
        for (std::size_t i = 0; i < cols; i++)
            for (std::size_t j = 0; j < rows; j++)
                accels[i] += jcb[i * rows + j] * lambda[j];
        return accels;
    }
}