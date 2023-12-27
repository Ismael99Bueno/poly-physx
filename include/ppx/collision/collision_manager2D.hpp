#pragma once

#include "kit/memory/scope.hpp"
#include "kit/utility/utils.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/collision/resolution/collision_resolution2D.hpp"

namespace ppx
{
class world2D;
class collision_manager2D
{
  public:
    collision_manager2D(world2D &world);
    world2D &world;

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
        if (m_collision_detection)
            ptr->epa_threshold = m_collision_detection->epa_threshold;

        m_collision_detection = std::move(coldet);
        m_collision_detection->world = &world;
        m_collision_detection->on_attach();
        return ptr;
    }
    template <typename T, class... ColSolvArgs> T *set_resolution(ColSolvArgs &&...args)
    {
        static_assert(std::is_base_of_v<collision_resolution2D, T>,
                      "Resolution method must inherit from collision_resolution2D");
        auto colres = kit::make_scope<T>(std::forward<ColSolvArgs>(args)...);
        T *ptr = colres.get();

        m_collision_resolution = std::move(colres);
        m_collision_resolution->world = &world;
        m_collision_resolution->on_attach();
        return ptr;
    }

    void solve();
    std::size_t size() const;

  private:
    kit::scope<collision_detection2D> m_collision_detection;
    kit::scope<collision_resolution2D> m_collision_resolution;
};
} // namespace ppx
