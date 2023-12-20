#pragma once

#include "kit/memory/scope.hpp"
#include "kit/utility/utils.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/collision/resolution/collision_resolution2D.hpp"

namespace ppx
{
class quad_tree_detection2D;
class brute_force_detection2D;
class sort_sweep_detection2D;

class spring_driven_resolution2D;
class constraint_driven_resolution2D;

class world2D;
class collision_manager2D
{
  public:
    enum class detection_type
    {
        BRUTE_FORCE = 0,
        QUAD_TREE = 1,
        SORT_AND_SWEEP = 2,
        CUSTOM = 3
    };
    enum class resolution_type
    {
        SPRING_DRIVEN = 0,
        CONSTRAINT_DRIVEN = 1,
        CUSTOM = 2
    };

    collision_manager2D(world2D &world);

    bool enabled = true;

    template <typename T = collision_detection2D> const T *detection() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_collision_detection);
    }
    template <typename T = collision_detection2D> T *detection()
    {
        return kit::get_casted_raw_ptr<T>(m_collision_detection);
    }
    template <typename T = collision_resolution2D> const T *resolution() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_collision_resolution);
    }
    template <typename T = collision_resolution2D> T *resolution()
    {
        return kit::get_casted_raw_ptr<T>(m_collision_resolution);
    }

    auto begin() const
    {
        return m_collision_detection->collisions().begin();
    }
    auto end() const
    {
        return m_collision_detection->collisions().end();
    }

    const collision2D &operator[](std::size_t index) const;

    template <typename T, class... ColDetArgs> T *set_detection(ColDetArgs &&...args)
    {
        static_assert(std::is_base_of_v<collision_detection2D, T>,
                      "Detection method must inherit from collision_detection2D");
        auto coldet = kit::make_scope<T>(std::forward<ColDetArgs>(args)...);
        T *ptr = coldet.get();
        m_collision_detection = std::move(coldet);
        m_collision_detection->world = &m_world;
        m_collision_detection->on_attach();

        if constexpr (std::is_same_v<T, quad_tree_detection2D>)
            m_det_type = detection_type::QUAD_TREE;
        else if constexpr (std::is_same_v<T, sort_sweep_detection2D>)
            m_det_type = detection_type::SORT_AND_SWEEP;
        else if constexpr (std::is_same_v<T, brute_force_detection2D>)
            m_det_type = detection_type::BRUTE_FORCE;
        else
            m_det_type = detection_type::CUSTOM;
        return ptr;
    }
    template <typename T, class... ColSolvArgs> T *set_resolution(ColSolvArgs &&...args)
    {
        static_assert(std::is_base_of_v<collision_resolution2D, T>,
                      "Resolution method must inherit from collision_resolution2D");
        auto coldet = kit::make_scope<T>(std::forward<ColSolvArgs>(args)...);
        T *ptr = coldet.get();
        m_collision_resolution = std::move(coldet);
        m_collision_resolution->world = &m_world;
        m_collision_resolution->on_attach();
        if constexpr (std::is_same_v<T, spring_driven_resolution2D>)
            m_res_type = resolution_type::SPRING_DRIVEN;
        else if constexpr (std::is_same_v<T, constraint_driven_resolution2D>)
            m_res_type = resolution_type::CONSTRAINT_DRIVEN;
        else
            m_res_type = resolution_type::CUSTOM;
        return ptr;
    }

    void solve();
    std::size_t size() const;

    detection_type detection_method() const;
    resolution_type resolution_method() const;

    collision_detection2D *detection(detection_type det_type);
    collision_resolution2D *resolution(resolution_type solv_type);

  private:
    world2D &m_world;

    kit::scope<collision_detection2D> m_collision_detection;
    kit::scope<collision_resolution2D> m_collision_resolution;

    detection_type m_det_type;
    resolution_type m_res_type;
};
} // namespace ppx
