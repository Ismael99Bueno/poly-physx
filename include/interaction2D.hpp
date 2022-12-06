#ifndef INTERACTION2D_HPP
#define INTERACTION2D_HPP

#include "entity_set.hpp"

namespace phys
{
    class interaction2D : public entity_set
    {
    public:
        using entity_set::entity_set;
        virtual std::pair<alg::vec2, float> force(const entity2D &e1, const entity2D &e2) const = 0;
    };
}
#endif