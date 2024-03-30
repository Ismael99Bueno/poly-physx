#pragma once

#include "ppx/collider/collider2D.hpp"
#include "ppx/collision/collision2D.hpp"

#include "ppx/collision/manifold/pp_manifold_algorithm2D.hpp"
#include "ppx/collision/detection/narrow/narrow_detection2D.hpp"

#include "ppx/internal/worldref.hpp"
#include "kit/memory/scope.hpp"
#include "kit/utility/utils.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/multithreading/mt_for_each.hpp"
#include "kit/container/hashable_tuple.hpp"

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 8
#endif

namespace ppx
{
class world2D;
class collision_detection2D : public worldref2D, kit::non_copyable
{
  public:
    using collision_map =
        std::unordered_map<kit::commutative_tuple<const collider2D *, const collider2D *>, collision2D>;

    collision_detection2D(world2D &world);
    virtual ~collision_detection2D() = default;

#ifdef KIT_PROFILE
    bool multithreaded = false;
#else
    bool multithreaded = true;
#endif

    template <kit::DerivedFrom<cp_narrow_detection2D> T = cp_narrow_detection2D> const T *cp_narrow_detection() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_cp_narrow);
    }
    template <kit::DerivedFrom<cp_narrow_detection2D> T = cp_narrow_detection2D> T *cp_narrow_detection()
    {
        return kit::get_casted_raw_ptr<T>(m_cp_narrow);
    }
    template <kit::DerivedFrom<pp_narrow_detection2D> T = pp_narrow_detection2D> const T *pp_narrow_detection() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_pp_narrow);
    }
    template <kit::DerivedFrom<pp_narrow_detection2D> T = pp_narrow_detection2D> T *pp_narrow_detection()
    {
        return kit::get_casted_raw_ptr<T>(m_pp_narrow);
    }

    template <kit::DerivedFrom<pp_manifold_algorithm2D> T = pp_manifold_algorithm2D>
    const T *pp_manifold_algorithm() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_pp_manifold);
    }
    template <kit::DerivedFrom<pp_manifold_algorithm2D> T = pp_manifold_algorithm2D> T *pp_manifold_algorithm()
    {
        return kit::get_casted_raw_ptr<T>(m_pp_manifold);
    }

    template <kit::DerivedFrom<cp_narrow_detection2D> T, class... NArgs>
    const T *set_cp_narrow_detection(NArgs &&...args)
    {
        auto nalg = kit::make_scope<T>(std::forward<NArgs>(args)...);
        T *ptr = nalg.get();

        m_cp_narrow = std::move(nalg);
        return ptr;
    }
    template <kit::DerivedFrom<pp_narrow_detection2D> T, class... NArgs>
    const T *set_pp_narrow_detection(NArgs &&...args)
    {
        auto nalg = kit::make_scope<T>(std::forward<NArgs>(args)...);
        T *ptr = nalg.get();

        m_pp_narrow = std::move(nalg);
        return ptr;
    }

    template <kit::DerivedFrom<pp_manifold_algorithm2D> T, class... ManifArgs>
    const T *set_pp_manifold_algorithm(ManifArgs &&...args)
    {
        auto manalg = kit::make_scope<T>(world, std::forward<ManifArgs>(args)...);
        T *ptr = manalg.get();

        m_pp_manifold = std::move(manalg);
        return ptr;
    }

    void remove_any_collisions_with(collider2D *collider);

    const collision_map &detect_collisions_cached();
    const collision_map &collisions() const;
    void flag_new_frame();

    virtual void on_attach()
    {
    }

    void inherit(collision_detection2D &&coldet);

  protected:
    void process_collision_st(collider2D *collider1, collider2D *collider2);
    void process_collision_mt(collider2D *collider1, collider2D *collider2, std::size_t thread_idx);
    void join_mt_collisions();

  private:
    collision_map m_collisions;
    collision_map m_last_collisions;
    std::array<collision_map, PPX_THREAD_COUNT> m_mt_collisions;

    bool m_new_frame = true;

    collision2D generate_collision(collider2D *collider1, collider2D *collider2) const;
    void cc_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;
    void cp_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;
    void pp_narrow_collision_check(collider2D *collider1, collider2D *collider2, collision2D &collision) const;

    void handle_collision_enter_exit_events() const;

    kit::scope<cp_narrow_detection2D> m_cp_narrow;
    kit::scope<pp_narrow_detection2D> m_pp_narrow;

    kit::scope<pp_manifold_algorithm2D> m_pp_manifold;

    virtual void detect_collisions() = 0;
};

} // namespace ppx
