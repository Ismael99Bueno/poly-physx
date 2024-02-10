#include "ppx/internal/pch.hpp"
#include "ppx/entities/collider2D.hpp"
#include "ppx/world2D.hpp"

namespace ppx
{
collider2D::collider2D(world2D &world, const body2D::ptr &parent, const specs &spc)
    : worldref2D(world), restitution(spc.restitution), friction(spc.friction), m_position(spc.position),
      m_parent(parent), m_density(spc.density), m_charge_density(spc.charge_density), m_type(spc.shape)
{
    const kit::transform2D<float> transform = kit::transform2D<float>::builder()
                                                  .position(spc.position)
                                                  .rotation(spc.rotation)
                                                  .parent(&parent->transform())
                                                  .build();
    switch (spc.shape)
    {
    case stype::POLYGON:
        m_shape = polygon(transform, spc.vertices);
        break;
    case stype::CIRCLE:
        m_shape = circle(transform, spc.radius);
        break;
    }
    update_parent();
}

collider2D::const_ptr collider2D::as_ptr() const
{
    return world.colliders.ptr(index);
}
collider2D::ptr collider2D::as_ptr()
{
    return world.colliders.ptr(index);
}

const body2D &collider2D::parent() const
{
    return *m_parent;
}
body2D &collider2D::parent()
{
    return *m_parent;
}

float collider2D::density() const
{
    return m_density;
}
void collider2D::density(float density)
{
    m_density = density;
}

float collider2D::charge_density() const
{
    return m_charge_density;
}
void collider2D::charge_density(float charge_density)
{
    m_charge_density = charge_density;
}

const aabb2D &collider2D::bounding_box() const
{
    return shape().bounding_box();
}
const kit::transform2D<float> &collider2D::transform() const
{
    return shape().transform();
}

const glm::vec2 &collider2D::lposition() const
{
    return m_position;
}
glm::vec2 collider2D::gposition() const
{
    return m_parent->props().position + m_position;
}

void collider2D::begin_update()
{
    KIT_ASSERT_ERROR(m_parent->m_spatial_update, "Cannot begin update collider while parent is not spatially updating")
    mutable_shape().begin_update();
}
void collider2D::end_update()
{
    mutable_shape().end_update();
}

float collider2D::area() const
{
    return shape().area();
}
float collider2D::inertia() const
{
    return shape().inertia();
}

bool collider2D::is_circle() const
{
    return m_type == stype::CIRCLE;
}
bool collider2D::is_polygon() const
{
    return m_type == stype::POLYGON;
}

void collider2D::translate(const glm::vec2 &dpos)
{
    mutable_shape().translate(dpos);
    m_position += dpos;
    update_parent();
}
void collider2D::rotate(float dangle)
{
    mutable_shape().rotate(dangle);
}

void collider2D::lposition(const glm::vec2 &position)
{
    mutable_shape().translate(position - m_position);
    m_position = position;
    update_parent();
}
void collider2D::gposition(const glm::vec2 &gposition)
{
    lposition(gposition - m_parent->props().position);
}

const glm::vec2 &collider2D::gcentroid() const
{
    return shape().centroid();
}
glm::vec2 collider2D::lcentroid() const
{
    return gcentroid() - m_parent->centroid();
}

void collider2D::gcentroid(const glm::vec2 &gcentroid)
{
    mutable_shape().centroid(gcentroid);
    update_parent();
}
void collider2D::lcentroid(const glm::vec2 &lcentroid)
{
    gcentroid(lcentroid + m_parent->centroid());
}

float collider2D::rotation() const
{
    return shape().transform().rotation;
}
const glm::vec2 &collider2D::origin() const
{
    return shape().transform().origin;
}

void collider2D::rotation(float rotation)
{
    mutable_shape().rotation(rotation);
}
void collider2D::origin(const glm::vec2 &origin)
{
    mutable_shape().origin(origin);
    update_parent();
}

const shape2D &collider2D::shape() const
{
    switch (m_type)
    {
    case stype::POLYGON:
        return std::get<polygon>(m_shape);
    case stype::CIRCLE:
        return std::get<circle>(m_shape);
    }
}
shape2D &collider2D::mutable_shape()
{
    switch (m_type)
    {
    case stype::POLYGON:
        return std::get<polygon>(m_shape);
    case stype::CIRCLE:
        return std::get<circle>(m_shape);
    }
}

void collider2D::update_parent()
{
    m_parent->update_centroids();
    m_parent->update_inertia();
    m_parent->update_colliders();
}

collider2D::stype collider2D::shape_type() const
{
    return m_type;
}

} // namespace ppx