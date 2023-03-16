#ifndef INTERACTION2D_HPP
#define INTERACTION2D_HPP

#include "entity2D_set.hpp"

namespace phys
{
    class interaction2D : public entity2D_set
    {
    public:
        using entity2D_set::entity2D_set;
        virtual ~interaction2D() = default;
        virtual std::pair<alg::vec2, float> force(const entity2D &e1, const entity2D &e2) const = 0;
        virtual float potential_energy_pair(const entity2D &e1, const entity2D &e2) const { return 0.f; }

        float potential(const entity2D &e, const alg::vec2 &pos) const;
        float potential(const alg::vec2 &pos) const;

        float potential_energy(const entity2D &e) const;
        float potential_energy() const;

        float energy(const entity2D &e) const;
        float energy() const;

    private:
        mutable phys::entity2D m_unit;
    };
}
#endif