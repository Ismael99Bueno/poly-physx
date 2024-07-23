#pragma once

#include "rk/integration/integrator.hpp"
#include "ppx/body/body_manager.hpp"
#include "ppx/collider/collider_manager.hpp"
#include "ppx/collision/collision_manager.hpp"
#include "ppx/behaviours/behaviour_manager.hpp"
#include "ppx/island/island_manager.hpp"
#include "ppx/joints/joint_repository.hpp"
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

    kit::mt::thread_pool *thread_pool = nullptr;

    void add(const specs::contraption2D &contraption);
    bool step();

    std::uint32_t step_count() const;
    std::uint32_t hertz() const;

    float rk_timestep() const;
    float substep_timestep() const;

    float kinetic_energy() const;
    float potential_energy() const;
    float energy() const;

    std::vector<float> operator()(float time, float timestep, const std::vector<float> &vars);

    void on_body_removal_validation(body2D *body);
    void add_builtin_joint_managers();

  private:
    std::uint32_t m_step_count = 0;
    std::uint32_t m_rk_substep_index = 0;
    float m_rk_timestep = 0.f;

    void pre_step();
    void post_step();
};

} // namespace ppx
