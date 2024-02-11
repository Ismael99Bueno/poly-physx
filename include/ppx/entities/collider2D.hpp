#pragma once

#include "ppx/entities/shapes2D.hpp"
#include "ppx/events/collider_events2D.hpp"
#include "ppx/entities/body2D.hpp"
#include <variant>

namespace ppx
{
class collider2D : public kit::identifiable<>, public kit::indexable, public worldref2D
{
  public:
    using ptr = kit::vector_ptr<collider2D>;
    using const_ptr = kit::const_vector_ptr<collider2D>;
    using specs = specs::collider2D;
    using stype = specs::stype;

    collider2D(world2D &world, const body2D::ptr &body, const specs &spc = {});

    float restitution;
    float friction;

    collider_events2D events;

    const_ptr as_ptr() const;
    ptr as_ptr();

    stype shape_type() const;

    const body2D &parent() const;
    body2D &parent();

    float area() const;
    float inertia() const;

    bool is_circle() const;
    bool is_polygon() const;

    float density() const;
    void density(float density);

    float charge_density() const;
    void charge_density(float charge_density);

    void begin_update();
    void end_update();

    void ltranslate(const glm::vec2 &dpos);
    void gtranslate(const glm::vec2 &dpos);
    void lrotate(float dangle);

    const aabb2D &bounding_box() const;
    const kit::transform2D<float> &ltransform() const;
    void ltransform(const kit::transform2D<float> &ltransform);

    const glm::vec2 &lposition() const;

    const glm::vec2 &gcentroid() const;
    glm::vec2 lcentroid() const;

    float lrotation() const;
    const glm::vec2 &origin() const;

    void lposition(const glm::vec2 &lposition);

    void gcentroid(const glm::vec2 &gcentroid);
    void lcentroid(const glm::vec2 &lcentroid);

    void lrotation(float lrotation);
    void origin(const glm::vec2 &origin);

    const shape2D &shape() const;
    template <kit::DerivedFrom<shape2D> T> const T &shape() const
    {
        return std::get<T>(m_shape);
    }
    template <kit::DerivedFrom<shape2D> T> const T *shape_if() const
    {
        return std::get_if<T>(&m_shape);
    }

    template <kit::DerivedFrom<shape2D> T, class... ShapeArgs> void set_shape(ShapeArgs &&...args)
    {
        T shape{std::forward<ShapeArgs>(args)...};
        shape.parent(&m_parent->centroid_transform());
        m_shape = shape;
        if constexpr (std::is_same_v<T, polygon>)
            m_type = stype::POLYGON;
        else if constexpr (std::is_same_v<T, circle>)
            m_type = stype::CIRCLE;
        update_parent();
    }

  private:
    std::variant<polygon, circle> m_shape;
    glm::vec2 m_position;
    body2D::ptr m_parent;
    float m_density;
    float m_charge_density;

    stype m_type;

    shape2D &mutable_shape();
    void update_parent();

    friend class body2D;
    friend class collider_manager2D;
};
} // namespace ppx