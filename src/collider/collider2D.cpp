#include "ppx/internal/pch.hpp"
#include "ppx/collider/collider2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
collider2D::collider2D(world2D &world, body2D *body, const specs &spc)
    : worldref2D(world), restitution(spc.props.restitution), friction(spc.props.friction),
      collision_filter(spc.props.collision_filter), m_position(spc.position), m_body(body),
      m_density(spc.props.density), m_charge_density(spc.props.charge_density), m_type(spc.props.shape)
{
    meta.index = world.colliders.size();
    transform2D transform{kit::transform2D<float>::builder().position(spc.position).rotation(spc.rotation).build()};
    transform.parent(&body->centroid_transform());

    switch (spc.props.shape)
    {
    case stype::POLYGON:
        m_shape = polygon(transform, spc.props.vertices);
        break;
    case stype::CIRCLE:
        m_shape = circle(transform, spc.props.radius);
        break;
    }
    // Parent must be updated (happens outside)
}

const geo::aabb2D &collider2D::tight_bbox() const
{
    return m_tight_bb;
}
const geo::aabb2D &collider2D::fat_bbox() const
{
    return m_fat_bb;
}

const body2D *collider2D::body() const
{
    return m_body;
}
body2D *collider2D::body()
{
    return m_body;
}

float collider2D::density() const
{
    return m_density;
}
void collider2D::density(float density)
{
    m_density = density;
    m_body->full_update();
}

float collider2D::charge_density() const
{
    return m_charge_density;
}
void collider2D::charge_density(float charge_density)
{
    m_charge_density = charge_density;
}

const transform2D &collider2D::ltransform() const
{
    return call_shape_method([](const auto &shape) -> const transform2D & { return shape.ltransform(); });
}
void collider2D::ltransform(const transform2D &ltransform)
{
    const glm::vec2 &lpos = call_shape_method([](const auto &shape) -> const glm::vec2 & { return shape.lposition(); });
    m_position += lpos - ltransform.position();
    call_shape_method([&ltransform](auto &shape) { shape.ltransform(ltransform); });
    m_body->full_update();
}

const glm::vec2 &collider2D::lposition() const
{
    return m_position;
}

void collider2D::begin_update()
{
    KIT_ASSERT_ERROR(m_body->spatial_updating(), "Cannot begin update collider while parent is not spatially updating")
    call_shape_method([](auto &shape) { shape.begin_update(); });
}
void collider2D::end_update()
{
    call_shape_method([](auto &shape) { shape.end_update(); });
    update_bounding_boxes();
}

float collider2D::area() const
{
    return call_shape_method([](const auto &shape) -> float { return shape.area(); });
}
float collider2D::inertia() const
{
    return call_shape_method([](const auto &shape) -> float { return shape.inertia(); });
}

bool collider2D::is_circle() const
{
    return m_type == stype::CIRCLE;
}
bool collider2D::is_polygon() const
{
    return m_type == stype::POLYGON;
}

void collider2D::ltranslate(const glm::vec2 &dpos)
{
    call_shape_method([&dpos](auto &shape) { shape.ltranslate(dpos); });
    m_position += dpos;
    m_body->full_update();
}
void collider2D::gtranslate(const glm::vec2 &dpos)
{
    call_shape_method([&dpos](auto &shape) { shape.gtranslate(dpos); });
    m_position += glm::vec2(m_body->centroid_transform().inv_ltransform() * glm::vec3(dpos, 0.f));
    m_body->full_update();
}
void collider2D::lrotate(float dangle)
{
    call_shape_method([&dangle](auto &shape) { shape.lrotate(dangle); });
    update_bounding_boxes();
}

void collider2D::lposition(const glm::vec2 &position)
{
    const glm::vec2 dpos = position - m_position;
    call_shape_method([&dpos](auto &shape) { shape.ltranslate(dpos); });

    m_position = position;
    m_body->full_update();
}

const glm::vec2 &collider2D::gcentroid() const
{
    return call_shape_method([](const auto &shape) -> const glm::vec2 & { return shape.gcentroid(); });
}
const glm::vec2 &collider2D::lcentroid() const
{
    return call_shape_method([](const auto &shape) -> const glm::vec2 & { return shape.lcentroid(); });
}

void collider2D::gcentroid(const glm::vec2 &gcentroid)
{
    const glm::vec2 &gc = call_shape_method([](const auto &shape) -> const glm::vec2 & { return shape.gcentroid(); });
    const glm::vec2 dpos = gcentroid - gc;
    call_shape_method([&gcentroid](auto &shape) { shape.gcentroid(gcentroid); });

    m_position += glm::vec2(m_body->centroid_transform().inv_ltransform() * glm::vec3(dpos, 0.f));
    m_body->full_update();
}
void collider2D::lcentroid(const glm::vec2 &lcentroid)
{
    ltranslate(lcentroid - shape().lcentroid());
}

float collider2D::lrotation() const
{
    return call_shape_method([](const auto &shape) -> float { return shape.lrotation(); });
}
const glm::vec2 &collider2D::origin() const
{
    return call_shape_method([](const auto &shape) -> const glm::vec2 & { return shape.origin(); });
}

void collider2D::lrotation(float lrotation)
{
    call_shape_method([&lrotation](auto &shape) { shape.lrotation(lrotation); });
    update_bounding_boxes();
}
void collider2D::origin(const glm::vec2 &origin)
{
    call_shape_method([&origin](auto &shape) { shape.origin(origin); });
    m_body->full_update();
}

const shape2D &collider2D::shape() const
{
    return std::visit<const shape2D &>([](const auto &shape) -> const shape2D & { return shape; }, m_shape);
}
shape2D &collider2D::mutable_shape()
{
    return std::visit<shape2D &>([](auto &shape) -> shape2D & { return shape; }, m_shape);
}

void collider2D::gtranslate_shape(const glm::vec2 &dpos)
{
    call_shape_method([&dpos](auto &shape) { shape.gtranslate(dpos); });
    update_bounding_boxes();
}

void collider2D::update_bounding_boxes()
{
    m_tight_bb =
        call_shape_method([](const auto &shape) -> const geo::aabb2D & { return shape.create_bounding_box(); });
    if (m_fat_bb.contains(m_tight_bb))
        return;

    const float enlargement = world.colliders.params.bbox_enlargement;
    KIT_ASSERT_ERROR(enlargement >= 0.f, "Bounding box expansion margin must be non-negative")

    auto qt = world.collisions.broad<quad_tree_broad2D>();
    if (qt)
        qt->erase(this);

    m_fat_bb = m_tight_bb;
    const glm::vec2 dpos = m_body->velocity() * enlargement;
    m_fat_bb.enlarge(dpos);
    if (qt)
        qt->insert(this);

    world.collisions.broad()->flag_update(this);
}

void collider2D::update_shape()
{
    mutable_shape().update();
    update_bounding_boxes();
}

collider2D::stype collider2D::shape_type() const
{
    return m_type;
}

} // namespace ppx