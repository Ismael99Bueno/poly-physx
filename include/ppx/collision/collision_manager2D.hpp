#pragma once

#include "kit/memory/ptr/scope.hpp"
#include "kit/utility/utils.hpp"
#include "kit/interface/toggleable.hpp"
#include "kit/utility/type_constraints.hpp"
#include "ppx/collision/contacts/collision_contacts2D.hpp"
#include "ppx/collision/contacts/contact_solver2D.hpp"
#include "ppx/collision/broad/quad_tree_broad2D.hpp"
#include "ppx/collision/broad/sort_sweep_broad2D.hpp"
#include "ppx/collision/broad/brute_force_broad2D.hpp"
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
        return m_broad->collisions().begin();
    }
    auto end() const
    {
        return m_broad->collisions().end();
    }

    const collision2D &operator[](std::size_t index) const;

    template <kit::DerivedFrom<broad_phase2D> T = broad_phase2D> const T *broad() const
    {
        return kit::get_casted_raw_ptr<const T>(m_broad, m_known_broads);
    }
    template <kit::DerivedFrom<broad_phase2D> T = broad_phase2D> T *broad()
    {
        return kit::get_casted_raw_ptr<T>(m_broad, m_known_broads);
    }

    template <kit::DerivedFrom<cp_narrow_phase2D> T = cp_narrow_phase2D> const T *cp_narrow() const
    {
        return kit::get_casted_raw_ptr<const T>(m_cp_narrow);
    }
    template <kit::DerivedFrom<cp_narrow_phase2D> T = cp_narrow_phase2D> T *cp_narrow()
    {
        return kit::get_casted_raw_ptr<T>(m_cp_narrow);
    }
    template <kit::DerivedFrom<pp_narrow_phase2D> T = pp_narrow_phase2D> const T *pp_narrow() const
    {
        return kit::get_casted_raw_ptr<const T>(m_pp_narrow);
    }
    template <kit::DerivedFrom<pp_narrow_phase2D> T = pp_narrow_phase2D> T *pp_narrow()
    {
        return kit::get_casted_raw_ptr<T>(m_pp_narrow);
    }

    template <kit::DerivedFrom<collision_contacts2D> T = collision_contacts2D> const T *contact_solver() const
    {
        return kit::get_casted_raw_ptr<const T>(m_contacts);
    }
    template <kit::DerivedFrom<collision_contacts2D> T = collision_contacts2D> T *contact_solver()
    {
        return kit::get_casted_raw_ptr<T>(m_contacts);
    }

    template <kit::DerivedFrom<broad_phase2D> T, class... ColDetArgs> T *set_broad(ColDetArgs &&...args)
    {
        auto broad = kit::make_scope<T>(world, std::forward<ColDetArgs>(args)...);
        if (m_broad)
            broad->inherit(std::move(*m_broad));

        T *ptr = broad.get();

        m_known_broads = {nullptr, nullptr, nullptr};
        if constexpr (kit::tuple_has_type_v<T *, known_broads_t>)
            std::get<T *>(m_known_broads) = ptr;

        m_broad = std::move(broad);
        m_broad->on_attach();
        return ptr;
    }

    template <kit::DerivedFrom<cp_narrow_phase2D> T, class... NArgs> const T *set_cp_narrow(NArgs &&...args)
    {
        auto nalg = kit::make_scope<T>(std::forward<NArgs>(args)...);
        T *ptr = nalg.get();

        m_cp_narrow = std::move(nalg);
        return ptr;
    }
    template <kit::DerivedFrom<pp_narrow_phase2D> T, class... NArgs> const T *set_pp_narrow(NArgs &&...args)
    {
        auto nalg = kit::make_scope<T>(std::forward<NArgs>(args)...);
        T *ptr = nalg.get();

        m_pp_narrow = std::move(nalg);
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

    kit::scope<broad_phase2D> m_broad;

    kit::scope<cp_narrow_phase2D> m_cp_narrow;
    kit::scope<pp_narrow_phase2D> m_pp_narrow;

    kit::scope<collision_contacts2D> m_contacts;

    using known_broads_t = std::tuple<quad_tree_broad2D *, sort_sweep_broad2D *, brute_force_broad2D *>;
    known_broads_t m_known_broads{nullptr, nullptr, nullptr};
    bool m_enabled = true;

    void set_constraint_based_contact_solver(contact_constraint_solver2D *contacts);
    void set_actuator_based_contact_solver(contact_actuator_solver2D *contacts);
    void detect_and_create_contacts();
    friend class world2D;
};
} // namespace ppx
