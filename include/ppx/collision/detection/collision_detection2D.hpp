#pragma once

#include "ppx/entities/collider2D.hpp"
#include "ppx/collision/collision2D.hpp"

#include "ppx/collision/manifold/manifold_algorithms2D.hpp"
#include "ppx/internal/worldref.hpp"
#include "kit/memory/scope.hpp"
#include "kit/utility/utils.hpp"
#include "kit/utility/type_constraints.hpp"
#include "kit/multithreading/mt_for_each.hpp"

#ifndef PPX_THREAD_COUNT
#define PPX_THREAD_COUNT 16
#endif

namespace ppx
{
class world2D;
class collision_detection2D : public worldref2D
{
  public:
    collision_detection2D(world2D &world);
    virtual ~collision_detection2D() = default;

    float epa_threshold = 1.e-3f;
#ifdef KIT_PROFILE
    bool multithreaded = false;
#else
    bool multithreaded = true;
#endif

    template <kit::DerivedFrom<cc_manifold_algorithm2D> T = cc_manifold_algorithm2D>
    const T *cc_manifold_algorithm() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_cc_manifold);
    }
    template <kit::DerivedFrom<cc_manifold_algorithm2D> T = cc_manifold_algorithm2D> T *cc_manifold_algorithm()
    {
        return kit::get_casted_raw_ptr<T>(m_cc_manifold);
    }
    template <kit::DerivedFrom<cp_manifold_algorithm2D> T = cp_manifold_algorithm2D>
    const T *cp_manifold_algorithm() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_cp_manifold);
    }
    template <kit::DerivedFrom<cp_manifold_algorithm2D> T = cp_manifold_algorithm2D> T *cp_manifold_algorithm()
    {
        return kit::get_casted_raw_ptr<T>(m_cp_manifold);
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

    template <kit::DerivedFrom<cc_manifold_algorithm2D> T, class... ManifArgs>
    const T *set_cc_manifold_algorithm(ManifArgs &&...args)
    {
        auto manalg = kit::make_scope<T>(std::forward<ManifArgs>(args)...);
        T *ptr = manalg.get();

        m_cc_manifold = std::move(manalg);
        return ptr;
    }
    template <kit::DerivedFrom<cp_manifold_algorithm2D> T, class... ManifArgs>
    const T *set_cp_manifold_algorithm(ManifArgs &&...args)
    {
        auto manalg = kit::make_scope<T>(std::forward<ManifArgs>(args)...);
        T *ptr = manalg.get();

        m_cp_manifold = std::move(manalg);
        return ptr;
    }
    template <kit::DerivedFrom<pp_manifold_algorithm2D> T, class... ManifArgs>
    const T *set_pp_manifold_algorithm(ManifArgs &&...args)
    {
        auto manalg = kit::make_scope<T>(std::forward<ManifArgs>(args)...);
        T *ptr = manalg.get();

        m_pp_manifold = std::move(manalg);
        return ptr;
    }

    const std::vector<collision2D> &detect_collisions_cached();
    void clear_cached_collisions();

    const std::vector<collision2D> &collisions() const;

    virtual void on_attach()
    {
    }

    void inherit(collision_detection2D &coldet);

  protected:
    void process_collision_st(collider2D &collider1, collider2D &collider2);
    void process_collision_mt(collider2D &collider1, collider2D &collider2, std::size_t thread_idx);
    void join_mt_collisions();

  private:
    std::vector<collision2D> m_collisions;
    std::array<std::vector<collision2D>, PPX_THREAD_COUNT> m_mt_collisions;
    kit::mt::feach_thread_pool<std::vector<collision2D>> m_pool{PPX_THREAD_COUNT};

    collision2D generate_collision(collider2D &collider1, collider2D &collider2) const;
    void cc_narrow_collision_check(collider2D &collider1, collider2D &collider2, collision2D &collision) const;
    void cp_narrow_collision_check(collider2D &collider1, collider2D &collider2, collision2D &collision) const;
    void pp_narrow_collision_check(collider2D &collider1, collider2D &collider2, collision2D &collision) const;

    void try_enter_or_stay_callback(const collision2D &c) const;
    void try_exit_callback(collider2D &collider1, collider2D &collider2) const;

    kit::scope<cc_manifold_algorithm2D> m_cc_manifold;
    kit::scope<cp_manifold_algorithm2D> m_cp_manifold;
    kit::scope<pp_manifold_algorithm2D> m_pp_manifold;

    virtual void detect_collisions() = 0;
};

} // namespace ppx
