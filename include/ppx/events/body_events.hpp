#ifndef PPX_BODY_EVENTS_HPP
#define PPX_BODY_EVENTS_HPP

#include "kit/utility/event.hpp"
#include "kit/utility/uuid.hpp"
#include "kit/memory/track_ptr.hpp"
#include <vector>
#include <functional>
#include <unordered_set>

namespace ppx
{
struct collision2D;
class body2D;
class body_events final
{
  public:
    void try_enter_or_stay(const collision2D &c) const;
    void try_exit(const kit::track_ptr<body2D> &incoming) const;

    kit::event<const collision2D &> on_collision_enter, on_collision_stay;
    kit::event<const kit::track_ptr<body2D> &> on_collision_exit;

  private:
    mutable std::unordered_set<kit::uuid> m_collided_ids;

    body_events(std::size_t allocations = 15);
    body_events(const body_events &) = default;
    body_events &operator=(const body_events &) = default;

    friend class body2D;
};
} // namespace ppx

#endif