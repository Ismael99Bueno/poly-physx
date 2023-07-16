#ifndef PPX_ENTITY_EVENTS_HPP
#define PPX_ENTITY_EVENTS_HPP

#include "kit/utility/event.hpp"
#include "kit/utility/uuid.hpp"
#include <vector>
#include <functional>
#include <unordered_set>

namespace ppx
{
struct collision2D;
class entity2D_ptr;
class entity_events final
{
  public:
    void try_enter_or_stay(const collision2D &c) const;
    void try_exit(const entity2D_ptr &incoming) const;

    kit::event<const collision2D &> on_collision_enter, on_collision_stay;
    kit::event<const entity2D_ptr &> on_collision_exit;

  private:
    mutable bool m_processed = false;
    mutable std::unordered_set<kit::uuid> m_collided_ids;

    void reset();

    entity_events(std::size_t allocations = 15);
    entity_events(const entity_events &) = default;
    entity_events &operator=(const entity_events &) = default;

    friend class entity2D;
    friend class engine2D;
};
} // namespace ppx

#endif