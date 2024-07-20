#pragma once

#include "rk/integration/integrator.hpp"
#include "ppx/body/body_manager2D.hpp"
#include "ppx/collider/collider_manager2D.hpp"
#include "ppx/collision/collision_manager2D.hpp"
#include "ppx/behaviours/behaviour_manager2D.hpp"
#include "ppx/island/island_manager2D.hpp"
#include "ppx/joints/joint_repository2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/utility/utils.hpp"
#include "kit/multithreading/thread_pool.hpp"

namespace ppx
{
class world2D : kit::non_copyable
{
  public:
    world2D(const specs::world2D &spc = {});

    rk::integrator<float> integrator;
    body_manager2D bodies;
    collider_manager2D colliders;
    joint_repository2D joints;
    behaviour_manager2D behaviours;
    collision_manager2D collisions;
    island_manager2D islands;

    bool semi_implicit_integration = true;
    kit::mt::thread_pool *thread_pool = nullptr;

    void add(const specs::contraption2D &contraption);
    std::uint32_t rk_substep_index() const;
    std::uint32_t rk_substeps() const;

    bool step();
    std::uint32_t step_count() const;

    float rk_substep_timestep() const;
    float timestep() const;
    std::uint32_t hertz() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    std::vector<float> operator()(float time, float timestep, const std::vector<float> &vars);

    void on_body_removal_validation(body2D *body);
    void add_builtin_joint_managers();

  private:
    float m_previous_timestep = 0.f;
    float m_rk_substep_timestep = integrator.ts.value;
    std::uint32_t m_rk_subset_index = 0;
    std::uint32_t m_step_count = 0;

    void pre_step_preparation();
    void post_step_setup();

    std::vector<float> create_state_derivative() const;
};

} // namespace ppx
