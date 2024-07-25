#pragma once

#include "rk/integration/state.hpp"
#include "ppx/collider/collider.hpp"
#include "ppx/manager.hpp"
#include "ppx/common/alias.hpp"
#include "kit/events/event.hpp"

namespace ppx
{
class collider_manager2D final : public contiguous_manager2D<collider2D>
{
  public:
    collider2D *add(body2D *parent, const collider2D::specs &spc = {});

    ray2D::hit<collider2D> cast(ray2D ray) const;
    void update_bounding_boxes();

    using contiguous_manager2D<collider2D>::operator[];
    std::vector<const collider2D *> operator[](const aabb2D &aabb) const;
    std::vector<collider2D *> operator[](const aabb2D &aabb);

    const collider2D *operator[](const glm::vec2 &point) const;
    collider2D *operator[](const glm::vec2 &point);

    using contiguous_manager2D<collider2D>::remove;
    bool remove(std::size_t index) override;

    specs::collider_manager2D params;

  private:
    using contiguous_manager2D<collider2D>::contiguous_manager2D;
};
} // namespace ppx
