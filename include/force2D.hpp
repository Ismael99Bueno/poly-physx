#ifndef FORCE2D_HPP
#define FORCE2D_HPP

#include "entity_set.hpp"

namespace phys
{
    class force2D : public entity_set
    {
    public:
        using entity_set::entity_set;
        virtual std::pair<alg::vec2, float> force(const entity2D &e) const = 0;
    };
}

#endif