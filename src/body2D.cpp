#include "ppx/internal/pch.hpp"
#include "ppx/body2D.hpp"
#include "ppx/behaviours/force2D.hpp"
#include "ppx/behaviours/interaction2D.hpp"

namespace ppx
{
body2D::body2D(const glm::vec2 &position, const glm::vec2 &velocity, const float rotation, const float angular_velocity,
               const float mass, const float charge, const bool kinematic)
    : m_shape(geo::polygon(position, rotation, geo::polygon::box(5.f))), m_vel(velocity), m_angvel(angular_velocity),
      m_mass(mass), m_inv_mass(1.f / m_mass), m_inertia(m_mass * shape().inertia()), m_inv_inertia(1.f / m_inertia),
      m_charge(charge), m_kinematic(kinematic)
{
}
body2D::body2D(const kit::block_vector<glm::vec2> &vertices, const glm::vec2 &position, const glm::vec2 &velocity,
               const float rotation, const float angular_velocity, const float mass, const float charge,
               const bool kinematic)
    : m_shape(geo::polygon(position, rotation, vertices)), m_vel(velocity), m_angvel(angular_velocity), m_mass(mass),
      m_inv_mass(1.f / m_mass), m_inertia(m_mass * shape().inertia()), m_inv_inertia(1.f / m_inertia), m_charge(charge),
      m_kinematic(kinematic)
{
}
body2D::body2D(const float radius, const glm::vec2 &position, const glm::vec2 &velocity, const float rotation,
               const float angular_velocity, const float mass, const float charge, const bool kinematic)
    : m_shape(geo::circle(position, radius, rotation)), m_vel(velocity), m_angvel(angular_velocity), m_mass(mass),
      m_inv_mass(1.f / m_mass), m_inertia(m_mass * shape().inertia()), m_inv_inertia(1.f / m_inertia), m_charge(charge),
      m_kinematic(kinematic)
{
}
body2D::body2D(const specs &spc)
    : m_vel(spc.velocity), m_angvel(spc.angular_velocity), m_mass(spc.mass), m_inv_mass(1.f / m_mass),
      m_charge(spc.charge), m_kinematic(spc.kinematic)
{
    if (spc.shape == shape_type::POLYGON)
        m_shape = geo::polygon(spc.position, spc.rotation, spc.vertices);
    else
        m_shape = geo::circle(spc.position, spc.radius, spc.rotation);
    compute_inertia(shape());
}

void body2D::retrieve(const std::vector<float> &vars_buffer)
{
    const std::size_t idx = 6 * index();
    geo::shape2D &sh = get_shape();

    sh.begin_update();
    sh.centroid({vars_buffer[idx + 0], vars_buffer[idx + 1]});
    sh.rotation(vars_buffer[idx + 2]);
    sh.end_update();

    m_vel = {vars_buffer[idx + 3], vars_buffer[idx + 4]};
    m_angvel = vars_buffer[idx + 5];
}

void body2D::retrieve()
{
    KIT_ASSERT_CRITICAL(m_state, "Trying to retrieve body data from a stateless body (the body is not tied to an "
                                 "engine -> its internal state is null)")
    retrieve(m_state->vars());
}
void body2D::dispatch() const
{
    const geo::shape2D &sh = shape();
    const glm::vec2 &position = sh.centroid();
    const float rotation = sh.rotation();
    rk::state &st = *m_state;

    const std::size_t idx = 6 * index();
    st[idx + 0] = position.x;
    st[idx + 1] = position.y;
    st[idx + 2] = rotation;
    st[idx + 3] = m_vel.x;
    st[idx + 4] = m_vel.y;
    st[idx + 5] = m_angvel;
}

float body2D::kinetic_energy() const
{
    return 0.5f * (m_mass * glm::length2(m_vel) + m_angvel * m_angvel * shape().inertia());
}

void body2D::add_force(const glm::vec2 &force)
{
    m_added_force += force;
}
void body2D::add_torque(const float torque)
{
    m_added_torque += torque;
}

const glm::vec2 &body2D::added_force() const
{
    return m_added_force;
}
float body2D::added_torque() const
{
    return m_added_torque;
}

const geo::shape2D &body2D::shape() const
{
    if (m_shape.index() == 0)
        return std::get<geo::polygon>(m_shape);
    return std::get<geo::circle>(m_shape);
}
geo::shape2D &body2D::get_shape()
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

void body2D::shape(const kit::block_vector<glm::vec2> &vertices)
{
    const geo::shape2D &sh = shape();
    const geo::polygon poly(sh.centroid(), sh.rotation(), vertices);
    compute_inertia(poly);
    m_shape = poly;
}
void body2D::shape(const float radius)
{
    const geo::shape2D &sh = shape();
    const geo::circle c(sh.centroid(), radius, sh.rotation());
    compute_inertia(c);
    m_shape = c;
}
void body2D::shape(const geo::polygon &poly)
{
    compute_inertia(poly);
    m_shape = poly;
}
void body2D::shape(const geo::circle &c)
{
    compute_inertia(c);
    m_shape = c;
}

void body2D::compute_inertia(const geo::shape2D &sh)
{
    m_inertia = m_mass * sh.inertia();
    m_inv_inertia = 1.f / m_inertia;
}

body2D::shape_type body2D::type() const
{
    return m_shape.index() == 0 ? shape_type::POLYGON : shape_type::CIRCLE;
}

float body2D::inertia() const
{
    return m_inertia;
}
float body2D::inverse_inertia() const
{
    return m_inv_inertia;
}

bool body2D::kinematic() const
{
    return m_kinematic;
}
void body2D::kinematic(const bool kinematic)
{
    m_kinematic = kinematic;
}

void body2D::translate(const glm::vec2 &dpos)
{
    get_shape().translate(dpos);
}
void body2D::rotate(const float dangle)
{
    get_shape().rotate(dangle);
}

const body_events &body2D::events() const
{
    return m_events;
}
body_events &body2D::events()
{
    return m_events;
}

const glm::vec2 &body2D::position() const
{
    return shape().centroid();
}
const glm::vec2 &body2D::velocity() const
{
    return m_vel;
}
const glm::vec2 body2D::vel_at(const glm::vec2 &at) const
{
    return m_vel + m_angvel * glm::vec2(-at.y, at.x);
}

float body2D::rotation() const
{
    return shape().rotation();
}
float body2D::angular_velocity() const
{
    return m_angvel;
}

float body2D::mass() const
{
    return m_mass;
}
float body2D::inverse_mass() const
{
    return m_inv_mass;
}
float body2D::charge() const
{
    return m_charge;
}

void body2D::position(const glm::vec2 &position)
{
    get_shape().centroid(position);
}
void body2D::velocity(const glm::vec2 &velocity)
{
    m_vel = velocity;
}

void body2D::rotation(const float rotation)
{
    get_shape().rotation(rotation);
}
void body2D::angular_velocity(const float angular_velocity)
{
    m_angvel = angular_velocity;
}

void body2D::mass(const float mass)
{
    m_mass = mass;
    m_inv_mass = 1.f / mass;
}
void body2D::charge(const float charge)
{
    m_charge = charge;
}

body2D::specs body2D::specs::from_body(const body2D &bd)
{
    if (const auto *poly = bd.shape_if<geo::polygon>())
        return {bd.position(), bd.velocity(),  bd.rotation(), bd.angular_velocity(), bd.mass(),
                bd.charge(),   poly->locals(), 0.f,           bd.kinematic(),        bd.type()};
    return {bd.position(),
            bd.velocity(),
            bd.rotation(),
            bd.angular_velocity(),
            bd.mass(),
            bd.charge(),
            {},
            bd.shape<geo::circle>().radius(),
            bd.kinematic(),
            bd.type()};
}

#ifdef KIT_USE_YAML_CPP
YAML::Emitter &operator<<(YAML::Emitter &out, const body2D &bd)
{
    out << YAML::BeginMap;
    out << YAML::Key << "UUID" << YAML::Value << (std::uint64_t)bd.id();
    out << YAML::Key << "Index" << YAML::Value << bd.index();
    out << YAML::Key << "Shape" << YAML::Value << bd.shape();
    out << YAML::Key << "Velocity" << YAML::Value << bd.velocity();
    out << YAML::Key << "Angular velocity" << YAML::Value << bd.angular_velocity();
    out << YAML::Key << "Mass" << YAML::Value << bd.mass();
    out << YAML::Key << "Charge" << YAML::Value << bd.charge();
    out << YAML::Key << "Kinematic" << YAML::Value << bd.kinematic();
    out << YAML::EndMap;
    return out;
}
#endif
} // namespace ppx

#ifdef KIT_USE_YAML_CPP
namespace YAML
{
Node convert<ppx::body2D>::encode(const ppx::body2D &bd)
{
    Node node;
    node["UUID"] = (std::uint64_t)bd.id();
    node["Index"] = bd.index();
    node["Shape"] = bd.shape();
    node["Velocity"] = bd.velocity();
    node["Angular velocity"] = bd.angular_velocity();
    node["Mass"] = bd.mass();
    node["Charge"] = bd.charge();
    node["Kinematic"] = bd.kinematic();
    return node;
}
bool convert<ppx::body2D>::decode(const Node &node, ppx::body2D &bd)
{
    if (!node.IsMap() || node.size() != 8)
        return false;

    bd.id(node["UUID"].as<std::uint64_t>());
    bd.index(node["Index"].as<std::size_t>());
    if (node["Shape"]["Radius"])
        bd.shape(node["Shape"].as<geo::circle>());
    else
        bd.shape(node["Shape"].as<geo::polygon>());
    bd.velocity(node["Velocity"].as<glm::vec2>());
    bd.angular_velocity(node["Angular velocity"].as<float>());
    bd.mass(node["Mass"].as<float>());
    bd.charge(node["Charge"].as<float>());
    bd.kinematic(node["Kinematic"].as<bool>());

    return true;
};
} // namespace YAML
#endif
