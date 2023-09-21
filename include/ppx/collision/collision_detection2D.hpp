#ifndef PPX_COLLISION_DETECTION2D_HPP
#define PPX_COLLISION_DETECTION2D_HPP

#include "ppx/body2D.hpp"

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 8
#endif

namespace ppx
{
class world2D;
struct collision2D
{
    body2D::ptr current;
    body2D::ptr incoming;

    glm::vec2 touch1{0.f};
    glm::vec2 touch2{0.f};
    glm::vec2 normal{0.f};

    bool valid = true;
};
class collision_detection2D
{
  public:
    virtual ~collision_detection2D() = default;

    virtual const std::vector<collision2D> &detect_collisions() = 0;
    const std::vector<collision2D> &cached_collisions();
    void flush_collisions();

  protected:
    std::vector<collision2D> m_collisions;
#ifdef PPX_MULTITHREADED
    std::array<std::vector<collision2D>, PPX_THREAD_COUNT> m_mt_collisions;
#endif
    world2D *m_parent = nullptr;

    bool gather_collision_data(const body2D &body1, const body2D &body2, collision2D *colis) const;
    bool narrow_collision_check(const body2D &body1, const body2D &body2, collision2D *colis) const;

    bool mixed_narrow_collision_check(const body2D &body1, const body2D &body2, collision2D *colis) const;
    bool circle_narrow_collision_check(const body2D &body1, const body2D &body2, collision2D *colis) const;

    void try_enter_or_stay_callback(const collision2D &c) const;
    void try_exit_callback(const body2D &body1, const body2D &body2) const;

  private:
    virtual void on_attach()
    {
    }

    friend class world2D;
};
} // namespace ppx

#endif