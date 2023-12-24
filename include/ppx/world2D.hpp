#pragma once

#include "rk/integrator.hpp"
#include "ppx/body_manager2D.hpp"
#include "ppx/collision/collision_manager2D.hpp"
#include "ppx/constraints/constraint_manager2D.hpp"
#include "ppx/behaviours/behaviour_manager2D.hpp"
#include "ppx/joints/spring_manager2D.hpp"
#include "ppx/events/world_events.hpp"
#include "kit/container/container_view.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
class world2D final : kit::non_copyable
{
  public:
#ifdef KIT_USE_YAML_CPP
    class serializer : public kit::serializer<world2D>
    {
      public:
        YAML::Node encode(const world2D &world) const override;
        bool decode(const YAML::Node &node, world2D &world) const override;
    };
#endif

    template <class... IntegArgs>
    world2D(IntegArgs &&...args)
        : integrator(std::forward<IntegArgs>(args)...), bodies(*this), springs(*this), behaviours(*this),
          collisions(*this), constraints(*this), m_previous_timestep(integrator.ts.value)
    {
    }

    rk::integrator<float> integrator;
    body_manager2D bodies;
    spring_manager2D springs;
    behaviour_manager2D behaviours;
    collision_manager2D collisions;
    constraint_manager2D constraints;
    world_events events;

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
