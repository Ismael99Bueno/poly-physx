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
    struct metrics
    {
        std::uint32_t total_collision_checks = 0;
        std::uint32_t positive_collision_checks = 0;
        float accuracy() const;
    };
    broad_phase2D(world2D &world);
    virtual ~broad_phase2D() = default;

    const std::vector<collision2D> &detect_collisions_cached(const cp_narrow_phase2D *cp_narrow,
                                                             const pp_narrow_phase2D *pp_narrow);
    const std::vector<collision2D> &collisions() const;
    void flush_collisions();

    virtual void on_attach()
    {
    }

    metrics collision_metrics() const;
    std::array<metrics, PPX_THREAD_COUNT> collision_metrics_per_thread() const;

    void inherit(broad_phase2D &&broad);

    specs::collision_manager2D::broad2D params;

  protected:
    void process_collision_st(collider2D *collider1, collider2D *collider2);
    void process_collision_mt(collider2D *collider1, collider2D *collider2, std::size_t thread_idx);
    void join_mt_collisions();

  private:
    std::vector<collision2D> m_collisions;
    std::array<std::vector<collision2D>, PPX_THREAD_COUNT> m_mt_collisions;

    metrics m_metrics;
    std::array<metrics, PPX_THREAD_COUNT> m_mt_metrics;

    const cp_narrow_phase2D *m_cp_narrow = nullptr;
    const pp_narrow_phase2D *m_pp_narrow = nullptr;

    collision2D generate_collision(collider2D *collider1, collider2D *collider2) const;
    void cc_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;
    void cp_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;
    void pp_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;

    virtual void detect_collisions() = 0;
};

} // namespace ppx
