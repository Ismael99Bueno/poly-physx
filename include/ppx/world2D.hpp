#pragma once

#include "rk/integration/integrator.hpp"
#include "ppx/entities/body_manager2D.hpp"
#include "ppx/entities/collider_manager2D.hpp"
#include "ppx/collision/collision_manager2D.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include "ppx/behaviours/behaviour_manager2D.hpp"
#include "ppx/joints/spring_manager2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
class world2D : kit::non_copyable
{
  public:
    template <class... IntegArgs>
    world2D(IntegArgs &&...args)
        : integrator(std::forward<IntegArgs>(args)...), bodies(*this), springs(*this), behaviours(*this),
          collisions(*this), constraints(*this), m_previous_timestep(integrator.ts.value)
    {
    }

    rk::integrator<float> integrator;
    body_manager2D bodies;
    collider_manager2D colliders;
    spring_manager2D springs;
    behaviour_manager2D behaviours;
    collision_manager2D collisions;
    constraint_manager2D constraints;

    bool semi_implicit_integration = false;

    bool step();
    float timestep_ratio() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    std::vector<float> operator()(float time, float timestep, const std::vector<float> &vars);

    void validate();

  private:
    float m_previous_timestep = 0.f;
    float m_timestep_ratio = 1.f;

    void pre_step_preparation();
    void post_step_setup();

    std::vector<float> create_state_derivative() const;
};

} // namespace ppx
