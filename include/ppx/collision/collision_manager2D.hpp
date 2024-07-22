#pragma once

#include "kit/memory/ptr/scope.hpp"
#include "kit/utility/utils.hpp"
#include "kit/utility/type_constraints.hpp"
#include "ppx/collision/contacts/icontact_manager2D.hpp"
#include "ppx/collision/broad/quad_tree_broad2D.hpp"
#include "ppx/collision/broad/brute_force_broad2D.hpp"
#include "ppx/collision/narrow/narrow_phase2D.hpp"
#include "ppx/internal/worldref.hpp"

namespace ppx
{
class world2D;

template <typename T>
concept ContactSolver2D =
    kit::DerivedFrom<T, icontact_constraint_manager2D> || kit::DerivedFrom<T, icontact_actuator_manager2D>;

class collision_manager2D final : public worldref2D, public kit::toggleable
{
  public:
    struct
    {
        kit::event<contact2D *> on_contact_enter;
        kit::event<contact2D *> on_contact_pre_solve;
        kit::event<contact2D *> on_contact_post_solve;
        kit::event<contact2D &> on_contact_exit;
    } events;

    template <kit::DerivedFrom<broad_phase2D> T = broad_phase2D> const T *broad() const
    {
        return kit::get_casted_raw_ptr<const T>(m_broad, m_known_broads);
    }
    template <kit::DerivedFrom<broad_phase2D> T = broad_phase2D> T *broad()
    {
        return kit::get_casted_raw_ptr<T>(m_broad, m_known_broads);
    }

    template <kit::DerivedFrom<narrow_phase2D> T = narrow_phase2D> const T *narrow() const
    {
        return kit::get_casted_raw_ptr<const T>(m_narrow);
    }
    template <kit::DerivedFrom<narrow_phase2D> T = narrow_phase2D> T *narrow()
    {
        return kit::get_casted_raw_ptr<T>(m_narrow);
    }

    template <kit::DerivedFrom<icontact_manager2D> T = icontact_manager2D> const T *contact_manager() const
    {
        return kit::get_casted_raw_ptr<const T>(m_contacts);
    }
    template <kit::DerivedFrom<icontact_manager2D> T = icontact_manager2D> T *contact_manager()
    {
        return kit::get_casted_raw_ptr<T>(m_contacts);
    }

    template <kit::DerivedFrom<broad_phase2D> T, class... ColDetArgs> T *set_broad(ColDetArgs &&...args)
    {
        auto broad = kit::make_scope<T>(world, std::forward<ColDetArgs>(args)...);
        if (m_broad)
            broad->params = m_broad->params;

        T *ptr = broad.get();

        m_known_broads = {nullptr, nullptr};
        if constexpr (kit::tuple_has_type_v<T *, known_broads_t>)
            std::get<T *>(m_known_broads) = ptr;

        m_broad = std::move(broad);
        return ptr;
    }

    template <kit::DerivedFrom<narrow_phase2D> T, class... NArgs> const T *set_narrow(NArgs &&...args)
    {
        auto narrow = kit::make_scope<T>(world, std::forward<NArgs>(args)...);
        if (m_narrow)
            narrow->params = m_narrow->params;

        T *ptr = narrow.get();

        m_narrow = std::move(narrow);
        return ptr;
    }

    template <ContactSolver2D T, class... SolvArgs> T *set_contact_manager(SolvArgs &&...args)
    {
        auto contacts = kit::make_scope<T>(world, std::forward<SolvArgs>(args)...);
        if (m_contacts)
            contacts->inherit(std::move(*m_contacts));

        T *ptr = contacts.get();
        if constexpr (kit::DerivedFrom<T, icontact_constraint_manager2D>)
            set_constraint_based_contact_manager(ptr);
        else if constexpr (kit::DerivedFrom<T, icontact_actuator_manager2D>)
            set_actuator_based_contact_manager(ptr);

        m_contacts = std::move(contacts);
        return ptr;
    }

    using kit::toggleable::enabled;
    void enabled(bool enable) override final;

  private:
    collision_manager2D(world2D &world);

    kit::scope<broad_phase2D> m_broad;
    kit::scope<narrow_phase2D> m_narrow;

    kit::scope<icontact_manager2D> m_contacts;

    using known_broads_t = std::tuple<quad_tree_broad2D *, brute_force_broad2D *>;
    known_broads_t m_known_broads{nullptr, nullptr};
    bool m_enabled = true;

    void set_constraint_based_contact_manager(icontact_constraint_manager2D *contacts);
    void set_actuator_based_contact_manager(icontact_actuator_manager2D *contacts);
    void detect_and_create_contacts();
    friend class world2D;
};
} // namespace ppx
