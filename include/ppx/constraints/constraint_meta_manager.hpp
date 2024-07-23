#pragma once

#include "ppx/joints/joint_meta_manager.hpp"
#include "ppx/constraints/constraint_manager.hpp"

namespace ppx
{
class icontact_constraint_manager2D;
class constraint_meta_manager2D final : public joint_meta_manager2D<iconstraint_manager2D>
{
  public:
    specs::joint_manager2D::constraints2D params;

  private:
    using joint_meta_manager2D<iconstraint_manager2D>::joint_meta_manager2D;
    void solve_velocities(std::vector<state2D> &states);
    void solve_positions(std::vector<state2D> &states);

    icontact_constraint_manager2D *m_contact_solver = nullptr;

    friend class world2D;
    friend class collision_manager2D;
};
} // namespace ppx