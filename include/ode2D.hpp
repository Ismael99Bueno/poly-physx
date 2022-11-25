#ifndef ODE2D_HPP
#define ODE2D_HPP

#include "engine2D.hpp"

namespace phys
{
    std::vector<float> ode(float t, const std::vector<float> &state, engine2D &engine);
}

#endif