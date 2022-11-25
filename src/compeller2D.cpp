#include "compeller2D.hpp"
#include "constrain_interface.hpp"

namespace phys
{
    compeller2D::compeller2D(std::vector<entity2D> &entities,
                             const std::size_t allocations) : m_entities(entities)
    {
        m_constrains.reserve(allocations);
    }

    void compeller2D::add_constrain(const constrain_interface &c) { m_constrains.push_back(&c); }

    void compeller2D::solve_and_load_constrains(std::vector<float> &stchanges, const std::vector<float> &inv_masses) const
    {
        const std::vector<float> jcb = jacobian(), djcb = jacobian_derivative();
        const std::vector<float> A = lhs(jcb, inv_masses);
        const std::vector<float> b = rhs(jcb, djcb, stchanges, inv_masses);
        const std::vector<float> lambda = lu_decomposition(A, b);
        return load_constrain_accels(jcb, lambda, stchanges);
    }

    std::vector<float> compeller2D::constrain_matrix(std::array<float, 3> (constrain_interface::*constrain_grad)(const entity_ptr &e) const) const
    {
        const std::size_t rows = m_constrains.size(), cols = 3 * m_entities.size();
        std::vector<float> cmatrix(rows * cols, 0.f);
        for (std::size_t i = 0; i < rows; i++)
            for (std::size_t j = 0; j < m_constrains[i]->size(); j++)
            {
                const entity_ptr &e = m_constrains[i]->operator[](j);
                const std::array<float, 3> state = (m_constrains[i]->*constrain_grad)(e);
                for (std::size_t k = 0; k < 3; k++)
                    cmatrix[i * cols + e.index() * 3 + k] = state[k];
            }
        return cmatrix;
    }

    std::vector<float> compeller2D::jacobian() const { return constrain_matrix(&constrain_interface::constrain_grad); }
    std::vector<float> compeller2D::jacobian_derivative() const { return constrain_matrix(&constrain_interface::constrain_grad_derivative); }

    std::vector<float> compeller2D::lhs(const std::vector<float> &jcb,
                                        const std::vector<float> &inv_masses) const
    {
        const std::size_t rows = m_constrains.size(), cols = 3 * m_entities.size();
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
        const std::size_t rows = m_constrains.size(), cols = 3 * m_entities.size();
        std::vector<float> b(rows, 0.f), qdot(stchanges.size() / 2, 0.f), accels(stchanges.size() / 2, 0.f);

        for (std::size_t i = 0; i < rows; i++)
        {
            for (std::size_t j = 0; j < m_entities.size(); j++)
                for (std::size_t k = 0; k < 3; k++)
                {
                    const std::size_t index1 = j * 3 + k, index2 = j * 6 + k;
                    const std::size_t id = i * cols + index1;
                    b[i] -= (djcb[id] * stchanges[index2] +
                             jcb[id] * stchanges[index2 + 3]) *
                            inv_masses[index1];
                }
            b[i] -= (m_constrains[i]->stiffness() * m_constrains[i]->value() +
                     m_constrains[i]->dampening() * m_constrains[i]->derivative());
        }

        // std::size_t index = 0;
        // for (std::size_t i = 0; i < m_entities.size(); i++)
        //     for (std::size_t j = 0; j < 3; j++)
        //     {
        //         qdot[index] = stchanges[VAR_PER_ENTITY * i + j];
        //         accels[index++] = stchanges[VAR_PER_ENTITY * i + j + 3];
        //     }

        // for (std::size_t i = 0; i < rows; i++)
        // {
        //     for (std::size_t j = 0; j < cols; j++)
        //     {
        //         const std::size_t id = i * cols + j;
        //         b[i] -= (djcb[id] * qdot[j] + jcb[id] * accels[j]);
        //     }
        //     b[i] -= (m_stiffness * m_constrains[i]->value() + m_dampening * m_constrains[i]->derivative());
        // }
        return b;
    }

    std::vector<float> compeller2D::lu_decomposition(const std::vector<float> &A,
                                                     const std::vector<float> &b) const
    {
        const std::size_t size = m_constrains.size();
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

    void compeller2D::load_constrain_accels(const std::vector<float> &jcb,
                                            const std::vector<float> &lambda,
                                            std::vector<float> &stchanges) const
    {
        const std::size_t rows = m_constrains.size();
        for (std::size_t i = 0; i < m_entities.size(); i++)
            if (m_entities[i].dynamic())
                for (std::size_t j = 0; j < 3; j++)
                    for (std::size_t k = 0; k < rows; k++)
                    {
                        const std::size_t id1 = 6 * i + j + 3,
                                          id2 = (i * 3 + j) * rows + k;
                        stchanges[id1] += jcb[id2] * lambda[k];
                    }
    }
}