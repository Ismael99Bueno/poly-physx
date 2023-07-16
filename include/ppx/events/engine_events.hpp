#ifndef PPX_ENGINE_EVENTS_HPP
#define PPX_ENGINE_EVENTS_HPP

#include "ppx/entity2D_ptr.hpp"
#include "ppx/constraints/constraint_interface2D.hpp"
#include "ppx/joints/spring2D.hpp"
#include "kit/utility/event.hpp"
#include "ppx/behaviours/force2D.hpp"
#include "ppx/behaviours/interaction2D.hpp"
#include <memory>
#include <cstdint>

namespace ppx
{
class engine_events final : kit::non_copyable
{
  public:
    kit::event<const entity2D_ptr &> on_entity_addition;
    kit::event<entity2D &> on_early_entity_removal;
    kit::event<std::size_t> on_late_entity_removal;
    kit::event<spring2D *> on_spring_addition;
    kit::event<const spring2D &> on_spring_removal;
    kit::event<constraint_interface2D *> on_constraint_addition;
    kit::event<const constraint_interface2D &> on_constraint_removal;
    kit::event<behaviour2D *> on_behaviour_addition;
    kit::event<const behaviour2D &> on_behaviour_removal;
};

} // namespace ppx

#endif