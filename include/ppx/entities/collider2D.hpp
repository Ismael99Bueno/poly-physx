#pragma once

#include "ppx/entities/body2D.hpp"
#include "ppx/events/collider_events2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/interface/indexable.hpp"
#include <variant>

namespace ppx
{
class body2D;
class collider2D : public kit::indexable, public worldref2D, kit::non_copyable
{
  public:
    using specs = specs::collider2D;
    using stype = specs::stype;

    collider2D(world2D &world, body2D *body, const specs &spc = {});

    float restitution;
    float friction;
    filter collision_filter;

    collider_events2D events;

    stype shape_type() const;

    const body2D *body() const;
    body2D *body();

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
        shape.parent(&m_body->centroid_transform());
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
    body2D *m_body;
    float m_density;
    float m_charge_density;

    stype m_type;

    shape2D &mutable_shape();
    void update_parent();

    friend class body2D;
    friend class collider_manager2D;
};
} // namespace ppx