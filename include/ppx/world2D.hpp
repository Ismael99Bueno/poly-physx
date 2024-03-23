#pragma once

#include "rk/integration/integrator.hpp"
#include "ppx/entities/body_manager2D.hpp"
#include "ppx/entities/collider_manager2D.hpp"
#include "ppx/collision/collision_manager2D.hpp"
#include "ppx/behaviours/behaviour_manager2D.hpp"
#include "ppx/joints/joint_repository2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/utility/utils.hpp"

namespace ppx
{
class world2D : kit::non_copyable
{
  public:
    template <class... IntegArgs>
    world2D(IntegArgs &&...args)
        : integrator(std::forward<IntegArgs>(args)...), bodies(*this), colliders(*this), joints(*this),
          behaviours(*this), collisions(*this), m_previous_timestep(integrator.ts.value)
    {
    }

    rk::integrator<float> integrator;
    body_manager2D bodies;
    collider_manager2D colliders;
    joint_repository2D joints;
    behaviour_manager2D behaviours;
    collision_manager2D collisions;
    struct
    {
        std::uint32_t iterations = 10;
        bool warmup = true;
        bool baumgarte_correction = true;

        float baumgarte_coef = 0.04f;
        float baumgarte_threshold = 0.005f;
    } constraints;

    bool semi_implicit_integration = false;

    void add(const specs::contraption2D &contraption);

    bool step();
    float timestep_ratio() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    std::vector<float> operator()(float time, float timestep, const std::vector<float> &vars);

    void on_body_addition_validation();
    void on_body_removal_validation();
    void add_builtin_joint_managers();

  private:
    float m_previous_timestep = 0.f;
    float m_timestep_ratio = 1.f;

    void pre_step_preparation();
    void post_step_setup();

    std::vector<float> create_state_derivative() const;
};

} // namespace ppx
