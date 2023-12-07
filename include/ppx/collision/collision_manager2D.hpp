#ifndef PPX_COLLISION_MANAGER2D_HPP
#define PPX_COLLISION_MANAGER2D_HPP

#include "kit/memory/scope.hpp"
#include "kit/utility/utils.hpp"
#include "ppx/collision/detection/quad_tree_detection2D.hpp"
#include "ppx/collision/solvers/spring_solver2D.hpp"

namespace ppx
{
class world2D;
class collision_manager2D
{
  public:
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
    template <typename T = collision_solver2D> const T *solver() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_collision_solver);
    }
    template <typename T = collision_solver2D> T *solver()
    {
        return kit::get_casted_raw_ptr<T>(m_collision_solver);
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
        return ptr;
    }
    template <typename T, class... ColSolvArgs> T *set_solver(ColSolvArgs &&...args)
    {
        static_assert(std::is_base_of_v<collision_solver2D, T>,
                      "Collision solver must inherit from collision_solver2D");
        auto coldet = kit::make_scope<T>(std::forward<ColSolvArgs>(args)...);
        T *ptr = coldet.get();
        m_collision_solver = std::move(coldet);
        m_collision_solver->world = &m_world;
        m_collision_detection->on_attach();
        return ptr;
    }

    void solve();
    std::size_t size() const;

  private:
    world2D &m_world;

    kit::scope<collision_detection2D> m_collision_detection;
    kit::scope<collision_solver2D> m_collision_solver;
};
} // namespace ppx

#endif