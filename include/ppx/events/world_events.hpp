#pragma once

#include "ppx/body2D.hpp"
#include "ppx/constraints/constraint2D.hpp"
#include "ppx/joints/spring2D.hpp"
#include "kit/event_handling/event.hpp"
#include "ppx/behaviours/force2D.hpp"
#include "ppx/behaviours/interaction2D.hpp"
#include <memory>
#include <cstdint>

namespace ppx
{
class world_events final : kit::non_copyable
{
  public:
    kit::event<const body2D::ptr &> on_body_addition;
    kit::event<const body2D &> on_early_body_removal;
    kit::event<std::size_t> on_late_body_removal;
    kit::event<const spring2D::ptr &> on_spring_addition;
    kit::event<const spring2D &> on_early_spring_removal;
    kit::event<std::size_t> on_late_spring_removal;
    kit::event<constraint2D *> on_constraint_addition;
    kit::event<const constraint2D &> on_constraint_removal;
    kit::event<behaviour2D *> on_behaviour_addition;
    kit::event<const behaviour2D &> on_behaviour_removal;
};

} // namespace ppx
