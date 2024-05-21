#pragma once

#include "rk/integration/integrator.hpp"
#include "ppx/body/body_manager2D.hpp"
#include "ppx/collider/collider_manager2D.hpp"
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
        std::uint32_t velocity_iterations = 8;
        std::uint32_t position_iterations = 3;
        bool warmup = true;
        bool baumgarte_correction = true;

        float baumgarte_coef = 0.035f;
        float baumgarte_threshold = 0.1f;
        float slop = 0.05f;

        float max_position_correction = 0.2f;
        float position_resolution_speed = 0.2f;
    } constraints;

    struct island_settings
    {
        island_settings(joint_repository2D &jr);

        bool enabled() const;
        void enabled(bool enable);

        float sleep_energy_threshold = 0.6f;
        float sleep_time_threshold = 1.f;

      private:
        joint_repository2D &m_joints;
    } islands{joints};

    bool semi_implicit_integration = true;

    void add(const specs::contraption2D &contraption);

    bool step();

    float rk_substep_timestep() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    std::vector<float> operator()(float time, float timestep, const std::vector<float> &vars);

    void on_body_removal_validation(body2D *body);
    void add_builtin_joint_managers();

  private:
    float m_previous_timestep = 0.f;
    float m_rk_substep_timestep = integrator.ts.value;

    void pre_step_preparation();
    void post_step_setup();

    std::vector<float> create_state_derivative() const;
};

} // namespace ppx
