#pragma once

#include "kit/memory/ptr/scope.hpp"
#include "kit/utility/utils.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/utility/type_constraints.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/collision/resolution/collision_resolution2D.hpp"
#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"
#include "ppx/collision/resolution/joint_driven_resolution2D.hpp"
#include "ppx/internal/worldref.hpp"

namespace ppx
{
class world2D;

template <typename T>
concept Resolution =
    kit::DerivedFrom<T, constraint_driven_resolution2D> || kit::DerivedFrom<T, joint_driven_resolution2D>;

class collision_manager2D : public kit::toggleable, public worldref2D
{
  public:
    struct
    {
        kit::event<contact2D *> on_contact_enter;
        kit::event<contact2D *> on_contact_pre_solve;
        kit::event<contact2D *> on_contact_post_solve;
        kit::event<contact2D &> on_contact_exit;
    } events;

    auto begin() const
    {
        return m_detection->collisions().begin();
    }
    auto end() const
    {
        return m_detection->collisions().end();
    }

    const collision2D &operator[](const kit::commutative_tuple<const collider2D *, const collider2D *> &key) const;
    const collision2D &at(const kit::commutative_tuple<const collider2D *, const collider2D *> &key) const;
    const collision2D &at(const collider2D *collider1, const collider2D *collider2) const;

    bool contains(const kit::commutative_tuple<const collider2D *, const collider2D *> &key) const;
    bool contains(const collider2D *collider1, const collider2D *collider2) const;

    auto find(const kit::commutative_tuple<const collider2D *, const collider2D *> &key) const
    {
        return m_detection->collisions().find(key);
    }
    auto find(const collider2D *collider1, const collider2D *collider2) const
    {
        return m_detection->collisions().find({collider1, collider2});
    }

    template <kit::DerivedFrom<collision_detection2D> T = collision_detection2D> const T *detection() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_detection);
    }
    template <kit::DerivedFrom<collision_detection2D> T = collision_detection2D> T *detection()
    {
        return kit::get_casted_raw_ptr<T>(m_detection);
    }
    template <kit::DerivedFrom<collision_resolution2D> T = collision_resolution2D> const T *resolution() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_resolution);
    }
    template <kit::DerivedFrom<collision_resolution2D> T = collision_resolution2D> T *resolution()
    {
        return kit::get_casted_raw_ptr<T>(m_resolution);
    }

    template <kit::DerivedFrom<collision_detection2D> T, class... ColDetArgs> T *set_detection(ColDetArgs &&...args)
    {
        auto coldet = kit::make_scope<T>(world, std::forward<ColDetArgs>(args)...);
        if (m_detection)
            coldet->inherit(std::move(*m_detection));

        T *ptr = coldet.get();

        m_detection = std::move(coldet);
        m_detection->on_attach();
        return ptr;
    }
    template <Resolution T, class... ColSolvArgs> T *set_resolution(ColSolvArgs &&...args)
    {
        auto colres = kit::make_scope<T>(world, std::forward<ColSolvArgs>(args)...);
        T *ptr = colres.get();
        if constexpr (kit::DerivedFrom<T, constraint_driven_resolution2D>)
            set_resolution_constraint_based(ptr);
        else if constexpr (kit::DerivedFrom<T, joint_driven_resolution2D>)
            set_resolution_non_constraint_based(ptr);

        m_resolution = std::move(colres);
        m_resolution->on_attach();
        return ptr;
    }

    std::size_t size() const;
    bool empty() const;

  private:
    collision_manager2D(world2D &world);

    kit::scope<collision_detection2D> m_detection;
    kit::scope<collision_resolution2D> m_resolution;

    void set_resolution_constraint_based(constraint_driven_resolution2D *resolution);
    void set_resolution_non_constraint_based(joint_driven_resolution2D *resolution);
    void detect_and_resolve();
    friend class world2D;
};
} // namespace ppx
