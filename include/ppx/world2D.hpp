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

    world2D(const rk::butcher_tableau &table = rk::butcher_tableau::rk4);

    rk::integrator integrator;
    body_manager2D bodies;
    spring_manager2D springs;
    behaviour_manager2D behaviours;
    collision_manager2D collisions;
    constraint_manager2D constraints;
    world_events events;

    bool raw_forward(float timestep);
    bool reiterative_forward(float &timestep, std::uint8_t reiterations = 2);
    bool embedded_forward(float &timestep);

    float current_timestep() const;
    float timestep_ratio() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    std::vector<float> operator()(float time, float timestep, const std::vector<float> &vars);

    float elapsed() const;
    void validate();

  private:
    float m_elapsed = 0.f;
    float m_current_timestep = 0.f;
    float m_timestep_ratio = 1.f;

    void pre_step_preparation(float timestep);
    void post_step_setup();

    std::vector<float> create_state_derivative() const;
};

} // namespace ppx
