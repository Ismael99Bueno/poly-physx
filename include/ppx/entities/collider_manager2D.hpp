#pragma once

#include "rk/integration/state.hpp"
#include "ppx/entities/collider2D.hpp"
#include "ppx/manager2D.hpp"
#include "kit/events/event.hpp"

namespace ppx
{
class collider_manager2D final : public manager2D<collider2D>
{
  public:
    using manager2D<collider2D>::manager2D;

    struct
    {
        kit::event<collider2D &> on_addition;
        kit::event<const collider2D &> on_early_removal;
        kit::event<std::size_t> on_late_removal;
    } events;

    collider2D &add(const body2D::ptr &parent, const collider2D::specs &spc = {});

    collider2D::const_ptr ptr(std::size_t index) const;
    collider2D::ptr ptr(std::size_t index);

    using manager2D<collider2D>::operator[];
    std::vector<collider2D::const_ptr> operator[](const aabb2D &aabb) const;
    std::vector<collider2D::ptr> operator[](const aabb2D &aabb);

    const collider2D *operator[](const glm::vec2 &point) const;
    collider2D *operator[](const glm::vec2 &point);

    using manager2D<collider2D>::remove;
    bool remove(std::size_t index) override;

    void validate();
    void validate_indices();
    void validate_parents();
};
} // namespace ppx
