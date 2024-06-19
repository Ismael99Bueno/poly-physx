#pragma once

#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/collider/collider_manager2D.hpp"

namespace ppx
{
class brute_force_broad2D final : public broad_phase2D
{
  public:
    using broad_phase2D::broad_phase2D;

  private:
    void detect_collisions() override;

    void detect_collisions_st();
    void detect_collisions_mt();
};
} // namespace ppx
