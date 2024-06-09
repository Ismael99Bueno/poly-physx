#pragma once

#include "kit/memory/ptr/scope.hpp"
#include "kit/utility/utils.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/utility/type_constraints.hpp"
#include "ppx/collision/detection/collision_detection2D.hpp"
#include "ppx/collision/contacts/collision_contacts2D.hpp"
#include "ppx/collision/contacts/contact_solver2D.hpp"
#include "ppx/internal/worldref.hpp"

namespace ppx
{
class world2D;

template <typename T>
concept ContactSolver2D =
    kit::DerivedFrom<T, contact_constraint_solver2D> || kit::DerivedFrom<T, contact_actuator_solver2D>;

class collision_manager2D : public worldref2D
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
    template <kit::DerivedFrom<collision_contacts2D> T = collision_contacts2D> const T *contacts() const
    {
        return kit::const_get_casted_raw_ptr<T>(m_contacts);
    }
    template <kit::DerivedFrom<collision_contacts2D> T = collision_contacts2D> T *contacts()
    {
        return kit::get_casted_raw_ptr<T>(m_contacts);
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
    template <ContactSolver2D T, class... SolvArgs> T *set_contact_solver(SolvArgs &&...args)
    {
        auto contacts = kit::make_scope<T>(world, std::forward<SolvArgs>(args)...);
        if (m_contacts)
            contacts->inherit(std::move(*m_contacts));

        T *ptr = contacts.get();
        if constexpr (kit::DerivedFrom<T, contact_constraint_solver2D>)
            set_constraint_based_contact_solver(ptr);
        else if constexpr (kit::DerivedFrom<T, contact_actuator_solver2D>)
            set_actuator_based_contact_solver(ptr);

        m_contacts = std::move(contacts);
        m_contacts->on_attach();
        return ptr;
    }

    std::size_t size() const;
    bool empty() const;
    bool enabled() const;
    void enabled(bool enable);

  private:
    collision_manager2D(world2D &world);

    kit::scope<collision_detection2D> m_detection;
    kit::scope<collision_contacts2D> m_contacts;
    bool m_enabled = true;

    void set_constraint_based_contact_solver(contact_constraint_solver2D *contacts);
    void set_actuator_based_contact_solver(contact_actuator_solver2D *contacts);
    void detect_and_create_contacts();
    friend class world2D;
};
} // namespace ppx
