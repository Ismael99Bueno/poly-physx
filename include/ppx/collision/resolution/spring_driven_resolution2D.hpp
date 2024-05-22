#pragma once

#include "ppx/collision/resolution/actuator_driven_resolution2D.hpp"
#include "ppx/collision/contacts/sd_contact2D.hpp"

namespace ppx
{
class spring_driven_resolution2D final : public actuator_driven_resolution2D
{
    using actuator_driven_resolution2D::actuator_driven_resolution2D;

    jd_contact_manager<sd_contact2D> m_contacts{world};

    void remove_any_contacts_with(const collider2D *collider) override;

    void solve() override;
    void resolve_contacts(const collision_detection2D::collision_map &collisions) override;
};
} // namespace ppx
