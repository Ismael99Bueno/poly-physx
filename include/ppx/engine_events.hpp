#ifndef ENGINE_EVENTS_HPP
#define ENGINE_EVENTS_HPP
#include "ppx/core.hpp"

#include "ppx/entity2D_ptr.hpp"
#include "ppx/constraint_interface2D.hpp"
#include "ppx/spring2D.hpp"
#include "ppx/event.hpp"
#include "ppx/force2D.hpp"
#include "ppx/interaction2D.hpp"
#include <memory>
#include <cstdint>

namespace ppx
{
    class engine_events final
    {
    public:
        engine_events() = default;
        event<const entity2D_ptr &> on_entity_addition;
        event<entity2D &> on_early_entity_removal;
        event<std::size_t> on_late_entity_removal;
        event<spring2D *> on_spring_addition;
        event<spring2D &> on_spring_removal;
        event<const ref<constraint_interface2D> &> on_constraint_addition, on_constraint_removal;
        event<const ref<behaviour2D> &> on_behaviour_addition, on_behaviour_removal;

        engine_events(const engine_events &) = delete;
        engine_events &operator=(const engine_events &) = delete;
    };

}

#endif