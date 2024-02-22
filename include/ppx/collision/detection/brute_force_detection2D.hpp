#pragma once

#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/entities/collider_manager2D.hpp"

namespace ppx
{
class brute_force_detection2D final : public collision_detection2D
{
  public:
    using collision_detection2D::collision_detection2D;

  private:
    kit::mt::feach_thread_pool<collider_manager2D> m_pool{PPX_THREAD_COUNT};
    void detect_collisions() override;

    void detect_collisions_st();
    void detect_collisions_mt();
};
} // namespace ppx
