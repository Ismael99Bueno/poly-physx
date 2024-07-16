#pragma once

#include "ppx/body/body2D.hpp"
#include "kit/interface/non_copyable.hpp"
#include "kit/events/event.hpp"
#include <variant>

namespace ppx
{
class contact2D;
class collider2D final : public worldref2D, kit::non_copyable
{
  public:
    using specs = specs::collider2D;
    using stype = specs::stype;

    collider2D(world2D &world, body2D *body, const specs &spc = {});

    float restitution;
    float friction;
    filter collision_filter;

    struct metadata
    {
        std::size_t index;
        bool broad_flag = false;
        std::size_t broad_index;
    } meta;

    struct
    {
        kit::event<contact2D *> on_contact_enter;
        kit::event<contact2D *> on_contact_pre_solve;
        kit::event<contact2D *> on_contact_post_solve;
        kit::event<contact2D &> on_contact_exit;
    } events;

    stype shape_type() const;

    const geo::aabb2D &tight_bbox() const;
    const geo::aabb2D &fat_bbox() const;

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

    const transform2D &ltransform() const;
    void ltransform(const transform2D &ltransform);

    const glm::vec2 &lposition() const;

    const glm::vec2 &gcentroid() const;
    const glm::vec2 &lcentroid() const;

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
        m_body->full_update();
    }

  private:
    std::variant<polygon, circle> m_shape;
    geo::aabb2D m_tight_bb;
    geo::aabb2D m_fat_bb;

    glm::vec2 m_position;
    body2D *m_body;
    float m_density;
    float m_charge_density;

    stype m_type;

    void gtranslate_shape(const glm::vec2 &dpos);

    void update_shape();
    void update_bounding_boxes();

    template <typename F> auto call_shape_method(F &&f) -> std::invoke_result_t<F, shape2D &>
    {
        return std::visit(std::forward<F>(f), m_shape);
    }
    template <typename F> auto call_shape_method_const(F &&f) const -> std::invoke_result_t<F, const shape2D &>
    {
        return std::visit(std::forward<F>(f), m_shape);
    }

    friend class body2D;
};
} // namespace ppx