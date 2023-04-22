#ifndef ODE2D_HPP
#define ODE2D_HPP

#include "ppx/engine2D.hpp"

namespace ppx
{
    std::vector<float> ode(float t, const std::vector<float> &vars, engine2D &engine);
}

#endif