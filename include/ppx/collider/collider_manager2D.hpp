#pragma once

#include "rk/integration/state.hpp"
#include "ppx/collider/collider2D.hpp"
#include "ppx/manager2D.hpp"
#include "kit/events/event.hpp"
#include "kit/memory/block_allocator.hpp"

namespace ppx
{
class collider_manager2D final : public manager2D<collider2D>
{
  public:
    collider2D *add(body2D *parent, const collider2D::specs &spc = {});

    using manager2D<collider2D>::operator[];
    std::vector<const collider2D *> operator[](const aabb2D &aabb) const;
    std::vector<collider2D *> operator[](const aabb2D &aabb);

    const collider2D *operator[](const glm::vec2 &point) const;
    collider2D *operator[](const glm::vec2 &point);

    using manager2D<collider2D>::remove;
    bool remove(std::size_t index) override;

  private:
    using manager2D<collider2D>::manager2D;
    kit::block_allocator<collider2D> m_allocator{1024};
};
} // namespace ppx
