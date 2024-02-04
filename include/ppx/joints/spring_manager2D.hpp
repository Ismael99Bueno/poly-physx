#pragma once

#include "ppx/joints/spring2D.hpp"
#include "ppx/manager2D.hpp"
#include "kit/events/event.hpp"

namespace ppx
{
class spring_manager2D : public manager2D<spring2D>
{
  public:
    using manager2D<spring2D>::manager2D;

    struct
    {
        kit::event<spring2D &> on_addition;
        kit::event<const spring2D &> on_early_removal;
        kit::event<std::size_t> on_late_removal;
    } events;

    template <class... SpringArgs> spring2D &add(SpringArgs &&...args)
    {
        spring2D &sp = m_elements.emplace_back(world, std::forward<SpringArgs>(args)...);
        process_addition(sp);
        return sp;
    }

    std::vector<spring2D::const_ptr> from_ids(kit::uuid id1, kit::uuid id2) const;
    std::vector<spring2D::ptr> from_ids(kit::uuid id1, kit::uuid id2);

    spring2D::const_ptr ptr(std::size_t index) const;
    spring2D::ptr ptr(std::size_t index);

    using manager2D<spring2D>::remove;
    bool remove(std::size_t index);

    void validate();

  private:
    void process_addition(spring2D &sp);

    void apply_forces();

    friend class world2D;
};
} // namespace ppx
