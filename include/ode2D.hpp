#ifndef ODE2D_HPP
#define ODE2D_HPP

#include "engine2D.hpp"

#define VAR_PER_ENTITY 6

namespace phys
{
    std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine);
}

#endif