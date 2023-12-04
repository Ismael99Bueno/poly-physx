#ifndef PPX_COLLISION_DETECTION2D_HPP
#define PPX_COLLISION_DETECTION2D_HPP

#include "ppx/body2D.hpp"
#include "ppx/collision/collision2D.hpp"

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 8
#endif

namespace ppx
{
class world2D;

class collision_detection2D
{
  public:
    virtual ~collision_detection2D() = default;

    world2D *world = nullptr;

    const std::vector<collision2D> &detect_collisions_cached();
    void clear_cached_collisions();

    std::size_t last_collision_count() const;

    virtual void on_attach()
    {
    }

  protected:
    std::vector<collision2D> m_collisions;
#ifdef PPX_MULTITHREADED
    std::array<std::vector<collision2D>, PPX_THREAD_COUNT> m_mt_collisions;
#endif

    bool gather_collision_data(body2D &body1, body2D &body2, collision2D *colis) const;
    bool narrow_collision_check(body2D &body1, body2D &body2, collision2D *colis) const;

    bool mixed_narrow_collision_check(body2D &body1, body2D &body2, collision2D *colis) const;
    bool circle_narrow_collision_check(body2D &body1, body2D &body2, collision2D *colis) const;

    void try_enter_or_stay_callback(const collision2D &c) const;
    void try_exit_callback(body2D &body1, body2D &body2) const;

  private:
    virtual void detect_collisions() = 0;

    std::size_t m_collision_count = 0;
};
} // namespace ppx

#endif