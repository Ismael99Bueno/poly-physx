#pragma once

#include "ppx/joints/joint2D.hpp"

namespace ppx
{
class actuator2D;
template <typename T>
concept IActuator2D = kit::DerivedFrom<T, actuator2D>;

template <typename T>
concept Actuator2D = Joint2D<T> && IActuator2D<T>;

class actuator2D : virtual public joint2D
{
  public:
    using joint2D::joint2D;

    bool is_constraint() const override;
    bool is_actuator() const override;

    void solve();

    virtual glm::vec4 compute_force() const = 0;
};
} // namespace ppx