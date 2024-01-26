#pragma once

#include "ppx/entities/shapes2D.hpp"
#include "ppx/internal/worldref.hpp"
#include "kit/interface/identifiable.hpp"
#include "kit/interface/indexable.hpp"
#include "kit/memory/vector_ptr.hpp"
#include <variant>

namespace ppx
{
class body2D;

class collider2D : public kit::identifiable<>, public kit::indexable, public worldref2D
{
  public:
    using ptr = kit::vector_ptr<collider2D>;
    using const_ptr = kit::const_vector_ptr<collider2D>;
    enum class shape_type
    {
        POLYGON = 0,
        CIRCLE = 1
    };
    struct specs
    {
        glm::vec2 position{0.f};
        float rotation = 0.f;
        float density = 1.f;
        float charge = 1.f;
        kit::dynarray<glm::vec2, PPX_MAX_VERTICES> vertices = polygon::square(5.f);
        float radius = 2.5f;
        shape_type shape = shape_type::POLYGON;
        static specs from_collider(const collider2D &collider);
    };

    collider2D(world2D &world, const polygon &poly);
    collider2D(world2D &world, const circle &circle);

    const_ptr as_ptr() const;
    ptr as_ptr();

  private:
    std::variant<polygon, circle> m_shape;
    kit::vector_ptr<body2D> m_parent;
};
} // namespace ppx