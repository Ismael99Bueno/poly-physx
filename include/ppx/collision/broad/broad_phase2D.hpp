#pragma once

#include "ppx/collider/collider2D.hpp"
#include "ppx/collision/collision2D.hpp"

#include "ppx/collision/narrow/narrow_phase2D.hpp"

#include "ppx/internal/worldref.hpp"
#include "kit/memory/ptr/scope.hpp"
#include "kit/utility/utils.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/multithreading/mt_for_each.hpp"
#include "kit/container/hashable_tuple.hpp"
#include "kit/interface/toggleable.hpp"

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 8
#endif

namespace ppx
{
class world2D;
class broad_phase2D : public worldref2D, public kit::toggleable, kit::non_copyable
{
  public:
    using collision_map =
        std::unordered_map<kit::commutative_tuple<const collider2D *, const collider2D *>, collision2D>;

    specs::collision_manager2D::detection2D params;

    broad_phase2D(world2D &world);
    virtual ~broad_phase2D() = default;

    const collision_map &detect_collisions_cached(const cp_narrow_phase2D *cp_narrow,
                                                  const pp_narrow_phase2D *pp_narrow);
    const collision_map &collisions() const;
    void update_last_collisions();

    virtual void on_attach()
    {
    }

    void inherit(broad_phase2D &&coldet);

  protected:
    void process_collision_st(collider2D *collider1, collider2D *collider2);
    void process_collision_mt(collider2D *collider1, collider2D *collider2, std::size_t thread_idx);
    void join_mt_collisions();

  private:
    collision_map m_collisions;
    collision_map m_last_collisions;
    std::array<collision_map, PPX_THREAD_COUNT> m_mt_collisions;

    const cp_narrow_phase2D *m_cp_narrow = nullptr;
    const pp_narrow_phase2D *m_pp_narrow = nullptr;

    collision2D generate_collision(collider2D *collider1, collider2D *collider2) const;
    void cc_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;
    void cp_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;
    void pp_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;

    virtual void detect_collisions() = 0;
};

} // namespace ppx
