#pragma once

#include "ppx/collision/resolution/constraint_driven_resolution2D.hpp"
#include "ppx/collision/contacts/si_contact2D.hpp"
#include "kit/container/hashable_tuple.hpp"

namespace ppx
{
class sequential_impulses_resolution2D final : public constraint_driven_resolution2D
{
    using constraint_driven_resolution2D::constraint_driven_resolution2D;
    cd_contact_manager<si_contact2D> m_contacts{world};

    void startup() override;
    void solve_velocities() override;
    bool solve_positions() override;
    void on_post_solve() override;

    void remove_any_contacts_with(const collider2D *collider) override;

    void resolve_contacts(const collision_detection2D::collision_map &collisions) override;
};
} // namespace ppx
