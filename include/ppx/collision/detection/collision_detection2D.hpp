#pragma once

#include "ppx/body2D.hpp"
#include "ppx/collision/collision2D.hpp"

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 8
#endif

namespace ppx
{
struct two_id_hash
{
    kit::uuid id1;
    kit::uuid id2;

    bool operator==(const two_id_hash &other) const
    {
        return id1 == other.id1 && id2 == other.id2;
    }
};
} // namespace ppx

namespace std
{
template <> struct hash<ppx::two_id_hash>
{
    size_t operator()(const ppx::two_id_hash &id) const;
};
} // namespace std

namespace ppx
{
class world2D;

class collision_detection2D
{
  public:
    virtual ~collision_detection2D() = default;

    world2D *world = nullptr;
    static inline bool build_contact_manifold_over_time = false;

    const std::vector<collision2D> &detect_collisions_cached();
    void clear_cached_collisions();
    void query_last_contact_points();

    const std::vector<collision2D> &collisions() const;

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

    std::unordered_map<two_id_hash, contact_point_query> m_last_contacts;
};

} // namespace ppx
