#ifndef PPX_BRUTE_FORCE_DETECTION2D_HPP
#define PPX_BRUTE_FORCE_DETECTION2D_HPP

#include "ppx/collision/detection/collision_detection2D.hpp"

namespace ppx
{
class brute_force_detection2D : public collision_detection2D
{
  public:
    using collision_detection2D::collision_detection2D;

  private:
    void detect_collisions() override;

    void detect_collisions_st();
    void detect_collisions_mt();
};
} // namespace ppx

#endif