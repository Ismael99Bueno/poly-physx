#ifndef FORCE2D_HPP
#define FORCE2D_HPP

#include "entity2D_set.hpp"

namespace phys
{
    class force2D : public entity2D_set
    {
    public:
        using entity2D_set::entity2D_set;
        virtual ~force2D() = default;
        virtual std::pair<alg::vec2, float> force(const entity2D &e) const = 0;
    };
}

#endif