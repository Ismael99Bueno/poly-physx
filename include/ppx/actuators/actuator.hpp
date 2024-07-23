#pragma once

#include "ppx/joints/joint.hpp"

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
    bool is_actuator() const override final;

    void solve(std::vector<state2D> &states);
    virtual glm::vec3 compute_force(const state2D &state1, const state2D &state2) const = 0;

    glm::vec2 reactive_force() const override final;
    float reactive_torque() const override final;

  private:
    glm::vec2 m_force{0.f};
    float m_torque = 0.f;
};
} // namespace ppx