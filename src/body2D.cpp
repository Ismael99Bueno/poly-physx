#include "ppx/internal/pch.hpp"
#include "ppx/body2D.hpp"
#include "ppx/world2D.hpp"
#include "ppx/behaviours/force2D.hpp"
#include "ppx/behaviours/interaction2D.hpp"

namespace ppx
{
body2D::body2D(const glm::vec2 &position, const glm::vec2 &velocity, const float rotation, const float angular_velocity,
               const float mass, const float charge, const bool kinematic)
    : kit::identifiable<>(kit::uuid::random()), velocity(velocity), angular_velocity(angular_velocity), charge(charge),
      kinematic(kinematic),
      m_shape(geo::polygon(kit::transform2D<float>::builder().position(position).rotation(rotation).build(),
                           geo::polygon::square(5.f))),
      m_mass(mass), m_inv_mass(1.f / m_mass), m_inertia(m_mass * shape<geo::polygon>().inertia()),
      m_inv_inertia(1.f / m_inertia)
{
}
body2D::body2D(const std::vector<glm::vec2> &vertices, const glm::vec2 &position, const glm::vec2 &velocity,
               const float rotation, const float angular_velocity, const float mass, const float charge,
               const bool kinematic)
    : kit::identifiable<>(kit::uuid::random()), velocity(velocity), angular_velocity(angular_velocity), charge(charge),
      kinematic(kinematic),
      m_shape(geo::polygon(kit::transform2D<float>::builder().position(position).rotation(rotation).build(), vertices)),
      m_mass(mass), m_inv_mass(1.f / m_mass), m_inertia(m_mass * shape<geo::polygon>().inertia()),
      m_inv_inertia(1.f / m_inertia)
{
}
body2D::body2D(const float radius, const glm::vec2 &position, const glm::vec2 &velocity, const float rotation,
               const float angular_velocity, const float mass, const float charge, const bool kinematic)
    : kit::identifiable<>(kit::uuid::random()), velocity(velocity), angular_velocity(angular_velocity), charge(charge),
      kinematic(kinematic),
      m_shape(geo::circle(kit::transform2D<float>::builder().position(position).rotation(rotation).build(), radius)),
      m_mass(mass), m_inv_mass(1.f / m_mass), m_inertia(m_mass * shape<geo::circle>().inertia()),
      m_inv_inertia(1.f / m_inertia)
{
}
body2D::body2D(const specs &spc)
    : kit::identifiable<>(kit::uuid::random()), velocity(spc.velocity), angular_velocity(spc.angular_velocity),
      charge(spc.charge), kinematic(spc.kinematic), m_mass(spc.mass), m_inv_mass(1.f / m_mass)
{
    if (spc.shape == shape_type::POLYGON)
    {
        m_shape = geo::polygon(kit::transform2D<float>::builder().position(spc.position).rotation(spc.rotation).build(),
                               spc.vertices);
        compute_inertia(shape<geo::polygon>());
    }
    else
    {
        m_shape = geo::circle(kit::transform2D<float>::builder().position(spc.position).rotation(spc.rotation).build(),
                              spc.radius);
        compute_inertia(shape<geo::circle>());
    }
}

void body2D::retrieve_data_from_state_variables(const std::vector<float> &vars_buffer)
{
    const std::size_t idx = 6 * index;
    geo::shape2D &sh = mutable_shape();

    sh.begin_update();
    sh.centroid({vars_buffer[idx + 0], vars_buffer[idx + 1]});
    sh.rotation(vars_buffer[idx + 2]);
    sh.end_update();

    velocity = {vars_buffer[idx + 3], vars_buffer[idx + 4]};
    angular_velocity = vars_buffer[idx + 5];
}

void body2D::reset_simulation_forces()
{
    m_force = glm::vec2(0.f);
    m_torque = 0.f;
}

body2D::const_ptr body2D::as_ptr() const
{
    return world->bodies.ptr(index);
}
body2D::ptr body2D::as_ptr()
{
    return world->bodies.ptr(index);
}

float body2D::kinetic_energy() const
{
    return 0.5f * (m_mass * glm::length2(velocity) + angular_velocity * angular_velocity * m_inertia);
}

void body2D::add_force_at(const glm::vec2 &force, const glm::vec2 &at)
{
    impulse_force += force;
    impulse_torque += kit::cross2D(at, force);
}

void body2D::apply_simulation_force(const glm::vec2 &force)
{
    m_force += force;
}
void body2D::apply_simulation_force_at(const glm::vec2 &force, const glm::vec2 &at)
{
    m_force += force;
    m_torque += kit::cross2D(at, force);
}
void body2D::apply_simulation_torque(const float torque)
{
    m_torque += torque;
}

const glm::vec2 &body2D::force() const
{
    return m_force;
}
float body2D::torque() const
{
    return m_torque;
}

const geo::shape2D &body2D::shape() const
{
    if (m_shape.index() == 0)
        return std::get<geo::polygon>(m_shape);
    return std::get<geo::circle>(m_shape);
}
geo::shape2D &body2D::mutable_shape()
{
    if (m_shape.index() == 0)
        return std::get<geo::polygon>(m_shape);
    return std::get<geo::circle>(m_shape);
}
template <typename T> const T &body2D::shape() const
{
    return std::get<T>(m_shape);
}

template const geo::polygon &body2D::shape<geo::polygon>() const;
template const geo::circle &body2D::shape<geo::circle>() const;

template <typename T> const T *body2D::shape_if() const
{
    return std::get_if<T>(&m_shape);
}

template const geo::polygon *body2D::shape_if<geo::polygon>() const;
template const geo::circle *body2D::shape_if<geo::circle>() const;

void body2D::shape(const std::vector<glm::vec2> &vertices)
{
    const geo::shape2D &sh = shape();
    const geo::polygon poly(sh.transform(), vertices);
    compute_inertia(poly);
    m_shape = poly;
}
void body2D::shape(const float radius)
{
    const geo::shape2D &sh = shape();
    const geo::circle circle(sh.transform(), radius);
    compute_inertia(circle);
    m_shape = circle;
}
void body2D::shape(const geo::polygon &poly)
{
    compute_inertia(poly);
    m_shape = poly;
}
void body2D::shape(const geo::circle &circle)
{
    compute_inertia(circle);
    m_shape = circle;
}

template <typename T> void body2D::compute_inertia(const T &shape)
{
    m_inertia = m_mass * shape.inertia();
    m_inv_inertia = 1.f / m_inertia;
}

bool body2D::is_polygon() const
{
    return m_shape.index() == 0;
}
bool body2D::is_circle() const
{
    return m_shape.index() == 1;
}
body2D::shape_type body2D::type() const
{
    return m_shape.index() == 0 ? shape_type::POLYGON : shape_type::CIRCLE;
}

float body2D::mass() const
{
    return kinematic ? m_mass : FLT_MAX;
}
float body2D::inv_mass() const
{
    return kinematic ? m_inv_mass : 0.f;
}

float body2D::inertia() const
{
    return kinematic ? m_inertia : FLT_MAX;
}
float body2D::inv_inertia() const
{
    return kinematic ? m_inv_inertia : 0.f;
}

float body2D::real_mass() const
{
    return m_mass;
}
float body2D::real_inv_mass() const
{
    return m_inv_mass;
}

float body2D::real_inertia() const
{
    return m_inertia;
}
float body2D::real_inv_inertia() const
{
    return m_inv_inertia;
}

void body2D::translate(const glm::vec2 &dpos)
{
    mutable_shape().translate(dpos);
}
void body2D::rotate(const float dangle)
{
    mutable_shape().rotate(dangle);
}

const kit::transform2D<float> &body2D::transform() const
{
    if (m_shape.index() == 0)
        return std::get<geo::polygon>(m_shape).transform();
    return std::get<geo::circle>(m_shape).transform();
}

const glm::vec2 &body2D::position() const
{
    return transform().position;
}

glm::vec2 body2D::velocity_at(const glm::vec2 &at) const
{
    return velocity + angular_velocity * glm::vec2(-at.y, at.x); // v + cross(w, at)
}
glm::vec2 body2D::constraint_velocity_at(const glm::vec2 &at) const
{
    return constraint_velocity + constraint_angular_velocity * glm::vec2(-at.y, at.x); // v + cross(w, at)
}

float body2D::rotation() const
{
    return transform().rotation;
}

void body2D::position(const glm::vec2 &position)
{
    mutable_shape().centroid(position);
}

void body2D::rotation(const float rotation)
{
    mutable_shape().rotation(rotation);
}

void body2D::mass(const float mass)
{
    m_mass = mass;
    m_inv_mass = 1.f / mass;
    compute_inertia(shape());
}

body2D::specs body2D::specs::from_body(const body2D &body)
{
    if (const auto *poly = body.shape_if<geo::polygon>())
    {
        const kit::transform2D<float> &transform = poly->transform();
        return {transform.position, body.velocity, transform.rotation,         body.angular_velocity,
                body.real_mass(),   body.charge,   poly->locals().as_vector(), 0.f,
                body.kinematic,     body.type()};
    }

    const geo::circle &circle = body.shape<geo::circle>();
    const kit::transform2D<float> &transform = circle.transform();
    return {transform.position,
            body.velocity,
            transform.rotation,
            body.angular_velocity,
            body.real_mass(),
            body.charge,
            {},
            circle.radius,
            body.kinematic,
            body.type()};
}
} // namespace ppx