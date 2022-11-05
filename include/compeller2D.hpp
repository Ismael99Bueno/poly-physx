#ifndef COMPELLER2D_HPP
#define COMPELLER2D_HPP

#include "entity_ptr.hpp"
#include <vector>

namespace physics
{
    class constrain2D;
    class compeller2D
    {
    public:
        compeller2D() = delete;
        compeller2D(const std::vector<entity2D> &entities, std::size_t allocations = 6);

        void add(const constrain2D *c); // Implement remove

        std::vector<float> solve_constrains(const std::vector<float> &state, const std::vector<float> &accels) const;

    private:
        const std::vector<entity2D> &m_entities;
        std::vector<const constrain2D *> m_constrains;

        std::vector<float> constrain_matrix(std::array<float, 3> (constrain2D::*constrain)(const std::vector<const_entity_ptr> &) const) const;
        std::vector<float> jacobian() const;
        std::vector<float> jacobian_derivative() const;

        std::vector<float> lhs(const std::vector<float> &jcb) const;
        std::vector<float> rhs(const std::vector<float> &jcb,
                               const std::vector<float> &djcb,
                               const std::vector<float> &state,
                               const std::vector<float> &stchanges) const;
        std::vector<float> lu_decomposition(const std::vector<float> &A, const std::vector<float> &b) const;
        std::vector<float> constrain_accels(const std::vector<float> &jcb, const std::vector<float> &lambda) const;
    };
}

#endif