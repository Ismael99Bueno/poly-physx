#pragma once

#include "ppx/joints/joint_meta_manager.hpp"
#include "ppx/actuators/actuator_manager.hpp"

namespace ppx
{
class icontact_actuator_manager2D;
class actuator_meta_manager2D final : public joint_meta_manager2D<iactuator_manager2D>
{
    using joint_meta_manager2D<iactuator_manager2D>::joint_meta_manager2D;
    icontact_actuator_manager2D *m_contact_solver = nullptr;

    void solve(std::vector<state2D> &states);
    friend class world2D;
    friend class collision_manager2D;
};
} // namespace ppx